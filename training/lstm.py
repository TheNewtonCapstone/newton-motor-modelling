import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Subset, random_split

from time_series_dataset import TimeSeriesDataset


class LSTMActuator(nn.Module):
    def __init__(self, input_size=2, hidden_size=32, num_layers=1, output_size=1):
        super(LSTMActuator, self).__init__()

        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        lstm_out, _ = self.lstm(x)  # Pass through LSTM
        output = self.fc(lstm_out)  # Apply fully connected layer
        return output


def train(model, dataloader, criterion, optimizer, device="cpu", num_epochs=10):
    model.to(device)  # Move model to GPU
    model.train()  # Set to training mode

    for epoch in range(num_epochs):
        total_loss = 0.0
        num_batches = 0

        for inputs, targets in dataloader:
            inputs, targets = (
                inputs.to(device),
                targets.to(device),
            )  # Move data to device (CPU/GPU)

            optimizer.zero_grad()  # Clear previous gradients
            outputs = model(inputs)  # Forward pass
            loss = criterion(outputs, targets)  # Compute loss
            loss.backward()  # Backpropagation
            optimizer.step()  # Update weights

            total_loss += loss.item()
            num_batches += 1

        avg_loss = total_loss / num_batches
        avg_root_loss = avg_loss ** 0.5
        print(f"Epoch {epoch + 1}/{num_epochs}, MSE: {avg_loss:.6f}, RMSE: {avg_root_loss:.6f}")


def evaluate(model, dataloader, criterion, device="cpu"):
    model.eval()  # Set to evaluation mode
    total_loss = 0.0
    num_batches = 0

    all_predictions = []
    all_targets = []

    with torch.no_grad():  # Disable gradient computation
        for inputs, targets in dataloader:
            inputs, targets = (
                inputs.to(device),
                targets.to(device),
            )  # Move to device (CPU/GPU)

            # Forward pass (get model predictions)
            outputs = model(inputs)

            # Compute loss
            loss = criterion(outputs, targets)
            total_loss += loss.item()
            num_batches += 1

            # Store predictions and actual values for further analysis
            all_predictions.append(outputs.cpu())  # Move to CPU for easy analysis
            all_targets.append(targets.cpu())

    # Compute average loss
    avg_loss = total_loss / num_batches

    # Convert lists to tensors
    all_predictions = torch.cat(all_predictions, dim=0)
    all_targets = torch.cat(all_targets, dim=0)

    print(f"Evaluation Loss: {avg_loss:.6f}")
    return avg_loss, all_predictions, all_targets


def main(model_path: str, data_path: str):
    # Evaluate model on validation set
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Instantiate model
    actuator_model = LSTMActuator(hidden_size=64, num_layers=2).to(device)

    # Define loss function & optimizer
    actuator_criterion = nn.MSELoss()
    actuator_optimizer = optim.Adam(actuator_model.parameters(), lr=0.001)

    dataset_transforms = {
        "position_error": ("target_position", "position", lambda x, y: x - y)
    }

    # Create DataLoader for batch training (and take 90% of data for training; 10% for validation)
    dataset = TimeSeriesDataset(
        data_path,
        input_columns=["position_error", "velocity"],
        target_columns=["torque"],
        seq_length=20,
        column_transforms=dataset_transforms,
    )

    # Perform random split
    train_dataset, eval_dataset = random_split(dataset, [0.9, 0.1])

    # Create DataLoaders (shuffle only batches, not individual sequences)
    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
    eval_loader = DataLoader(eval_dataset, batch_size=64, shuffle=False)

    # Training loop
    train(
        actuator_model,
        train_loader,
        actuator_criterion,
        actuator_optimizer,
        device=device,
        num_epochs=50,
    )

    torch.save(actuator_model.state_dict(), model_path)

    actuator_model.eval()

    # Evaluate model on validation set
    evaluate(actuator_model, eval_loader, actuator_criterion, device=device)


def main_eval(model_path: str, data_path: str):
    # Evaluate model on validation set
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Load model
    model = LSTMActuator().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))

    # Define column transformations
    dataset_transforms = {
        "position_error": ("target_position", "position", lambda x, y: x - y)
    }

    # Create DataLoader for batch training (and take 90% of data for training; 10% for validation)
    dataset = TimeSeriesDataset(
        data_path,
        input_columns=["position_error", "velocity"],
        target_columns=["torque"],
        seq_length=150,
        column_transforms=dataset_transforms,
    )

    # Define split sizes
    train_size = int(0.9 * len(dataset))  # 90% of the dataset
    eval_size = len(dataset) - train_size  # Remaining 10%

    # Perform random split
    train_dataset, eval_dataset = random_split(dataset, [train_size, eval_size])

    # Create DataLoaders (shuffle only batches, not individual sequences)
    eval_loader = DataLoader(eval_dataset, batch_size=40, shuffle=False)

    # Evaluate model on validation set
    evaluate(model, eval_loader, nn.MSELoss(), device=device)


def cli(model_path: str):
    # Load the model
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = LSTMActuator().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))

    from training.motor_controller import MotorController

    motor_controller = MotorController(model, device)

    print("Motor Controller CLI - Enter a target position to get torque predictions.")
    print("Type 'exit' to quit.\n")

    while True:
        try:
            target_pos = input("Enter target position: ")
            if target_pos.lower() == "exit":
                print("Exiting...")
                break

            target_pos = float(target_pos)
            predicted_torque = motor_controller.predict_torque(target_pos)

            print(f"Predicted Torque: {predicted_torque:.4f}")
            print(f"Updated Position: {motor_controller.current_position:.4f}")
            print(f"Updated Velocity: {motor_controller.current_velocity:.4f}\n")

        except ValueError:
            print("Invalid input! Please enter a valid number or type 'exit'.")


if __name__ == "__main__":
    import subprocess
    import os

    git_root = (
        subprocess.check_output("git rev-parse --show-toplevel", shell=True)
        .decode("utf-8")
        .strip()
    )
    model_path = os.path.join(git_root, "models", "lstm_motor_model.pth")
    data_path = os.path.join(git_root, "data", "data_full_sin.csv")

    main(model_path, data_path)
