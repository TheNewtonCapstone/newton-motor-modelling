import time
from threading import Thread

import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split

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


class WeightedMSELoss(nn.Module):
    def __init__(self, threshold=0.8):
        super().__init__()
        self.threshold = threshold

    def forward(self, pred, target):
        # Higher weights for high torque regions
        weights = torch.where(torch.abs(target) > self.threshold, 2.0, 1.0)
        return torch.mean(weights * (pred - target) ** 2)


class TrainingThread(Thread):
    def __init__(self, model, train_dataloader, eval_dataloader, criterion, optimizer, scheduler, device="cpu", num_epochs=100, early_stop_patience=10):
        super().__init__()

        self.model = model
        self.train_dataloader = train_dataloader
        self.eval_dataloader = eval_dataloader
        self.criterion = criterion
        self.optimizer = optimizer
        self.scheduler = scheduler
        self.device = device
        self.num_epochs = num_epochs
        self.early_stop_patience = early_stop_patience

        self.losses = []
        self.rmse_losses = []
        self.learning_rates = []
        self.eval_losses = []
        self.eval_rmse_losses = []
        self.train_eval_loss_diffs = []

        self.best_mse_loss = float('inf')
        self.patience_counter = 0

    def run(self):
        self.model.to(self.device)

        for epoch in range(self.num_epochs):
            self.model.train()

            total_loss = 0.0
            num_batches = 0

            for inputs, targets in self.train_dataloader:
                inputs, targets = inputs.to(self.device), targets.to(self.device)
                self.optimizer.zero_grad()
                outputs = self.model(inputs)
                loss = self.criterion(outputs, targets)
                loss.backward()
                self.optimizer.step()

                total_loss += loss.item()
                num_batches += 1

            mse_loss = total_loss / num_batches
            self.scheduler.step(mse_loss)
            current_lr = self.optimizer.param_groups[0]['lr']

            eval_loss, eval_rmse_loss, _, _ = self.evaluate()
            eval_loss_diff = mse_loss - eval_loss

            self.losses.append(mse_loss)
            self.rmse_losses.append(mse_loss ** 0.5)
            self.learning_rates.append(current_lr)
            self.eval_losses.append(eval_loss)
            self.eval_rmse_losses.append(eval_rmse_loss)
            self.train_eval_loss_diffs.append(eval_loss_diff)

            print(f"Epoch {epoch + 1}/{self.num_epochs}, MSE: {mse_loss:.6f}, LR: {current_lr:.6f}, Eval MSE: {eval_loss:.6f}, Eval RMSE: {eval_rmse_loss:.6f}, Diff: {eval_loss_diff:.6f}")

            if mse_loss < self.best_mse_loss:
                self.best_mse_loss = mse_loss
                self.patience_counter = 0
            else:
                self.patience_counter += 1

            if self.patience_counter >= self.early_stop_patience:
                print(f"Early stopping at epoch {epoch}")
                break

    def evaluate(self):
        self.model.eval()  # Set to evaluation mode
        total_loss = 0.0
        num_batches = 0

        all_predictions = []
        all_targets = []

        with torch.no_grad():  # Disable gradient computation
            for inputs, targets in self.eval_dataloader:
                inputs, targets = (
                    inputs.to(self.device),
                    targets.to(self.device),
                )  # Move to device (CPU/GPU)

                # Forward pass (get model predictions)
                outputs = self.model(inputs)

                # Compute loss
                loss = self.criterion(outputs, targets)
                total_loss += loss.item()
                num_batches += 1

                # Store predictions and actual values for further analysis
                all_predictions.append(outputs.cpu())  # Move to CPU for easy analysis
                all_targets.append(targets.cpu())

        # Compute average loss
        mse_loss = total_loss / num_batches
        rmse_loss = mse_loss ** 0.5

        # Convert lists to tensors
        all_predictions = torch.cat(all_predictions, dim=0)
        all_targets = torch.cat(all_targets, dim=0)

        return mse_loss, rmse_loss, all_predictions, all_targets


def plot(train_thread: TrainingThread):
    plt.clf()

    plt.subplot(2, 1, 1)
    plt.plot(train_thread.losses, label='MSE', color='blue')
    plt.plot(train_thread.rmse_losses, label='RMSE', color='red')
    plt.plot(train_thread.eval_losses, label='Eval MSE', color='orange')
    plt.plot(train_thread.eval_rmse_losses, label='Eval RMSE', color='purple')
    plt.plot(train_thread.train_eval_loss_diffs, label='Train-Eval Diff', color='black')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(train_thread.learning_rates, label='Learning Rate', color='green')
    plt.xlabel('Epoch')
    plt.ylabel('Learning Rate')
    plt.yscale('log')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.gcf().canvas.draw_idle()
    plt.gcf().canvas.start_event_loop(0.1)


def main(model_path: str, data_path: str):
    # Evaluate model on validation set
    device = "cuda" if torch.cuda.is_available() else "cpu"

    early_stop_patience = 6

    # Instantiate model
    actuator_model = LSTMActuator(hidden_size=32, num_layers=1).to(device)

    # Define loss function & optimizer
    actuator_criterion = WeightedMSELoss()
    actuator_optimizer = optim.Adam(actuator_model.parameters(), lr=0.001)
    actuator_scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        actuator_optimizer,
        mode='min',
        factor=0.3,
        patience=early_stop_patience // 2,
        min_lr=1e-6
    )

    dataset_transforms = {
        "position_error_rad": ("target_position", "position", lambda x, y: (x - y) * 2 * torch.pi),
        "velocity_rad": ("velocity", "velocity", lambda x, y: x * 2 * torch.pi),
    }

    # Create DataLoader for batch training (and take 90% of data for training; 10% for validation)
    dataset = TimeSeriesDataset(
        data_path,
        input_columns=["position_error_rad", "velocity_rad"],
        target_columns=["torque"],
        seq_length=4,
        column_transforms=dataset_transforms,
    )

    # Perform random split
    train_dataset, eval_dataset = random_split(dataset, [0.9, 0.1])

    # Create DataLoaders (shuffle only batches, not individual sequences)
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    eval_loader = DataLoader(eval_dataset, batch_size=32, shuffle=False)

    # Training loop
    train_thread = TrainingThread(
        actuator_model,
        train_loader,
        eval_loader,
        actuator_criterion,
        actuator_optimizer,
        actuator_scheduler,
        device=device,
        num_epochs=200,
        early_stop_patience=early_stop_patience,
    )
    train_thread.start()

    plt.ion()
    plt.figure(figsize=(10, 10))

    while train_thread.is_alive():
        plot(train_thread)
        time.sleep(0.1)

    plt.ioff()
    train_thread.join()

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

    # Perform random split
    train_dataset, eval_dataset = random_split(dataset, [0.9, 0.1])

    # Create DataLoaders (shuffle only batches, not individual sequences)
    eval_loader = DataLoader(eval_dataset, batch_size=40, shuffle=False)

    # Evaluate model on validation set
    evaluate(model, eval_loader, nn.MSELoss(), device=device)


if __name__ == "__main__":
    import subprocess
    import os

    git_root = (
        subprocess.check_output("git rev-parse --show-toplevel", shell=True)
        .decode("utf-8")
        .strip()
    )
    model_path = os.path.join(git_root, "models", "lstm_motor_model.pth")
    data_path = os.path.join(git_root, "data", "complete", "data_full_sin.csv")

    main(model_path, data_path)
