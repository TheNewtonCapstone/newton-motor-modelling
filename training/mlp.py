# actuator_model.py
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import pandas as pd
import numpy as np
from typing import Tuple, Dict
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import os
os.environ['MPLBACKEND'] = 'Agg'  # Force matplotlib to not use any Xwindows backend



class ActuatorDataset(Dataset):
    def __init__(self, csv_path: str,
                 history_steps: int = 3,
                 target_freq: int = 100):
        """
        Args:
            csv_path: Path to CSV file
            history_steps: Number of history steps (paper uses 3: t, t-0.01, t-0.02)
        """
        raw_df = pd.read_csv(csv_path)
        self.target_dt = 1 / target_freq

        measured_dts = np.diff(raw_df["time"].values)
        mean_dt = np.mean(measured_dts)
        current_freq = 1 / mean_dt
        downsample_factor = int( current_freq/ target_freq)
        print(f"Mean dt: {np.mean(mean_dt):.4f}, Std dt: {np.std(measured_dts):.4f}")
        print(f"Downsample factor: {downsample_factor}")

        # downsample the data
        self.df = raw_df[::downsample_factor].reset_index(drop=True)
        measured_dts = np.diff(self.df["time"].values)
        mean_dt = np.mean(measured_dts)
        print(f"Mean dt: {np.mean(mean_dt):.4f}, Std dt: {np.std(measured_dts):.4f}")


        self.data = None  # Will hold processed DataFrame
        self.history_steps = history_steps
        self.scaler_params = {}  # Store normalization parameters
        self.features = None
        self.targets = None

    def preprocess_data(self) -> None:
        """Load and preprocess data"""
        self.df["position_error"] = self.df["target_position"] - self.df["position"]
        features = []
        targets = []

        for i in range(len(self.df) - self.history_steps + 1):
            window = self.df.iloc[i:i + self.history_steps]

            # Check if window is valid
            time_diffs = np.diff(window["time"].values)

            if np.any(abs(time_diffs - self.target_dt) > 0.002) :
                continue
            # extract features and targets
            pos_errors = window["position_error"].values
            velocities = window["velocity"].values
            feature = np.concatenate([pos_errors, velocities])
            target = window["torque"].values[-1]
            features.append(feature)
            targets.append(target)
        self.features = np.array(features)[::-1]
        self.targets = np.array(targets)

    def normalize_data(self) -> None:
        """Normalize features"""
        pos_errors = self.features[:, :self.history_steps]
        velocities = self.features[:, self.history_steps:]

        # store normalization parameters
        self.scaler_params = {
            "pos_error_mean": pos_errors.mean(),
            "pos_error_std": pos_errors.std(),
            "pos_error_max": pos_errors.max(),
            "pos_error_min": pos_errors.min(),
            "vel_mean": velocities.mean(),
            "vel_std": velocities.std(),
            "vel_max": velocities.max(),
            "vel_min": velocities.min(),
            "torque_mean": self.targets.mean(),
            "torque_std": self.targets.std(),
            "torque_max": self.targets.max(),
            "torque_min": self.targets.min()
        }

        # Standardize data (z-score normalization)
        pos_errors_norm = (pos_errors - self.scaler_params['pos_error_mean']) / self.scaler_params['pos_error_std']
        velocities_norm = (velocities - self.scaler_params['vel_mean']) / self.scaler_params['vel_std']
        targets_norm = (self.targets - self.scaler_params['torque_mean']) / self.scaler_params['torque_std']

        self.features = np.concatenate([pos_errors_norm, velocities_norm], axis=1)
        self.targets = targets_norm
        print(f"Features shape: {self.features.shape}, Targets shape: {self.targets.shape}")

    def save_preprocessed_data(self, file_path: str) -> None:
        headers = ["pos_error_t-0.000", "pos_error_t-0.010", "pos_error_t-0.020", "velocity_t-0.000",
                   "velocity_t-0.010", "velocity_t-0.020", "torque"]
        processed_data: np.array = np.column_stack([self.features, self.targets])
        df_processed_data = pd.DataFrame(processed_data)
        df_processed_data.to_csv(file_path, index=False, header=headers, float_format='%.6f')

    def __len__(self) -> int:
        """Return length of dataset"""
        return len(self.features)

    def save_csv(self, path: str) -> None:
        data_dict = {
            f'pos_error_t-{i * self.target_dt:.3f}': self.features[:, i] * self.scaler_params['pos_error_std'] +
                                                     self.scaler_params['pos_error_mean']
            for i in range(self.history_steps)
        }

        # Add velocities
        data_dict.update({
            f'velocity_t-{i * self.target_dt:.3f}': self.features[:, i + self.history_steps] * self.scaler_params[
                'vel_std'] + self.scaler_params['vel_mean']
            for i in range(self.history_steps)
        })

        # Add target (torque)
        data_dict['torque'] = self.targets * self.scaler_params['torque_std'] + self.scaler_params['torque_mean']

        # Create DataFrame and save
        df = pd.DataFrame(data_dict)
        df.to_csv(path, index=False)

        # Save normalization parameters separately
        param_path = path.rsplit('.', 1)[0] + '_params.csv'
        pd.DataFrame([self.scaler_params]).to_csv(param_path, index=False)


    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """Get item from dataset"""
        return self.features[idx], self.targets[idx]


class ActuatorNet(nn.Module):

    def __init__(self, input_size: int = 6, hidden_size: int = 32, num_layers: int = 3):
        super().__init__()
        # input layer
        layers = [
            nn.Linear(input_size, hidden_size),
            nn.Softsign()
        ]

        # hidden layers
        for _ in range(num_layers - 2):
            layers.extend([
                nn.Linear(hidden_size, hidden_size),
                nn.Softsign()
            ])
            layers.append(nn.Linear(hidden_size, 1))

        self.network = nn.Sequential(*layers)
        self.initialize_weights()

    def initialize_weights(self) -> None:
        """Initialize weights"""
        for layer in self.network:
            if isinstance(layer, nn.Linear):
                nn.init.xavier_uniform_(layer.weight)
                nn.init.zeros_(layer.bias)
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass"""
        return self.network(x)

    def predict(self, pos_error: torch.Tensor, vel_error: torch.Tensor) -> torch.Tensor:
        """Convenience method to predict"""
        x = torch.cat([pos_error, vel_error], dim=1)
        return self.forward(x)


class ActuatorTrainer:
    """Handles training of the actuator model"""

    def __init__(self, model: ActuatorNet, device: str = 'cuda' if torch.cuda.is_available() else 'cpu'):
        """
        Args:
            model: ActuatorNet instance
            device: Device to train on
        """
        self.model = model
        self.device = device

    def compute_metrics(self, loader: DataLoader) -> Dict[str, float]:
        """Compute detailed metrics on given data loader"""
        self.model.eval()
        all_targets = []
        all_predictions = []

        with torch.no_grad():
            for features, targets in loader:
                features = features.to(self.device).float()
                targets = targets.to(self.device).float()

                outputs = self.model(features)

                all_targets.extend(targets.cpu().numpy())
                all_predictions.extend(outputs.squeeze(1).cpu().numpy())

        all_targets = np.array(all_targets)
        all_predictions = np.array(all_predictions)

        return {
            'mse': mean_squared_error(all_targets, all_predictions),
            'rmse': np.sqrt(mean_squared_error(all_targets, all_predictions)),
            'mae': mean_absolute_error(all_targets, all_predictions),
            'r2': r2_score(all_targets, all_predictions)
        }

    def plot_training_history(self, history: Dict, save_path: str = None):
        """Plot training history"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

        # Plot losses
        ax1.plot(history['train_loss'], label='Train Loss')
        ax1.plot(history['val_loss'], label='Validation Loss')
        ax1.set_xlabel('Epoch')
        ax1.set_ylabel('Loss')
        ax1.set_title('Training and Validation Loss')
        ax1.legend()
        ax1.grid(True)

        # Plot learning rate
        ax2.plot(history['learning_rates'])
        ax2.set_xlabel('Epoch')
        ax2.set_ylabel('Learning Rate')
        ax2.set_title('Learning Rate Schedule')
        ax2.grid(True)
        ax2.set_yscale('log')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path)
        plt.show()

    def plot_prediction_vs_actual(self, loader: DataLoader, save_path: str = None):
        """Plot prediction vs actual values"""
        self.model.eval()
        all_targets = []
        all_predictions = []

        with torch.no_grad():
            for features, targets in loader:
                features = features.to(self.device).float()
                targets = targets.to(self.device).float()

                outputs = self.model(features)

                all_targets.extend(targets.cpu().numpy())
                all_predictions.extend(outputs.squeeze(1).cpu().numpy())

        plt.figure(figsize=(10, 10))
        plt.scatter(all_targets, all_predictions, alpha=0.5)
        plt.plot([-2, 2], [-2, 2], 'r--')  # Perfect prediction line
        plt.xlabel('Actual Values')
        plt.ylabel('Predicted Values')
        plt.title('Prediction vs Actual')
        plt.grid(True)

        if save_path:
            plt.savefig(save_path)
        plt.show()

    def train(self,
              train_loader: DataLoader,
              val_loader: DataLoader,
              epochs: int = 100,
              lr: float = 1e-3,
              patience: int = 10) -> Dict:
        """Train the actuator model

        Args:
            train_loader: Training data loader
            val_loader: Validation data loader
            epochs: Number of epochs to train
            lr: Initial learning rate
            patience: Number of epochs to wait for improvement before early stopping

        Returns:
            Dict containing training history
        """
        self.model = self.model.to(self.device)
        criterion = nn.MSELoss()
        optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)
        scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5)

        # Initialize tracking variables
        best_val_loss = float('inf')
        patience_counter = 0
        history = {
            'train_loss': [],
            'val_loss': [],
            'learning_rates': []
        }
        best_model_state = None

        for epoch in range(epochs):
            # Training phase
            self.model.train()
            train_losses = []

            for features, targets in train_loader:
                features = features.to(self.device).float()
                targets = targets.to(self.device).float()

                # Forward pass
                outputs = self.model(features)
                loss = criterion(outputs, targets.unsqueeze(1))

                # Backward pass and optimize
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                train_losses.append(loss.item())

            # Validation phase
            self.model.eval()
            val_losses = []

            with torch.no_grad():
                for features, targets in val_loader:
                    features = features.to(self.device).float()
                    targets = targets.to(self.device).float()

                    outputs = self.model(features)
                    val_loss = criterion(outputs, targets.unsqueeze(1))
                    val_losses.append(val_loss.item())

            # Calculate average losses
            avg_train_loss = np.mean(train_losses)
            avg_val_loss = np.mean(val_losses)
            current_lr = optimizer.param_groups[0]['lr']

            # Update history
            history['train_loss'].append(avg_train_loss)
            history['val_loss'].append(avg_val_loss)
            history['learning_rates'].append(current_lr)

            # Print progress
            print(
                f'Epoch [{epoch + 1}/{epochs}] Train Loss: {avg_train_loss:.4f} Val Loss: {avg_val_loss:.4f} LR: {current_lr:.6f}')

            # Learning rate scheduling
            scheduler.step(avg_val_loss)

            # Early stopping check
            if avg_val_loss < best_val_loss:
                best_val_loss = avg_val_loss
                patience_counter = 0
                best_model_state = self.model.state_dict().copy()
            else:
                patience_counter += 1

            if patience_counter >= patience:
                print(f'Early stopping triggered after {epoch + 1} epochs')
                break

        # Restore best model
        if best_model_state is not None:
            self.model.load_state_dict(best_model_state)

        return history

    def denormalize_and_compare(self, loader: DataLoader, dataset: ActuatorDataset, base_save_path: str = "comparison"):
        """
        Denormalize predictions and actual values, save to CSV and create separate plots
        """
        self.model.eval()
        results = {
            'position_error': [],
            'velocity': [],
            'actual_torque': [],
            'predicted_torque': []
        }

        with torch.no_grad():
            for features, targets in loader:
                features = features.to(self.device).float()
                outputs = self.model(features)

                # Denormalize predictions and targets
                pred_denorm = (outputs.squeeze().cpu().numpy() * dataset.scaler_params['torque_std'] +
                               dataset.scaler_params['torque_mean'])
                target_denorm = (targets.cpu().numpy() * dataset.scaler_params['torque_std'] +
                                 dataset.scaler_params['torque_mean'])

                # Denormalize position error and velocity (taking just current timestep)
                pos_error_denorm = (features[:, 0].cpu().numpy() * dataset.scaler_params['pos_error_std'] +
                                    dataset.scaler_params['pos_error_mean'])
                vel_denorm = (features[:, 3].cpu().numpy() * dataset.scaler_params['vel_std'] +
                              dataset.scaler_params['vel_mean'])

                # Collect results
                results['position_error'].extend(pos_error_denorm)
                results['velocity'].extend(vel_denorm)
                results['actual_torque'].extend(target_denorm)
                results['predicted_torque'].extend(pred_denorm)

        # Save to CSV
        df = pd.DataFrame(results)
        df.to_csv(f"{base_save_path}.csv", index=False)

        # Save denormalization factors
        with open(f"{base_save_path}_denorm_factors.txt", 'w') as f:
            f.write("Denormalization Factors:\n")
            for key, value in dataset.scaler_params.items():
                f.write(f"{key}: {value}\n")

        # Create separate plots with plt.figure()

        # 1. Actual vs Predicted Torque
        plt.figure(figsize=(10, 8))
        plt.scatter(results['actual_torque'], results['predicted_torque'], alpha=0.5, s=1)
        min_val = min(min(results['actual_torque']), min(results['predicted_torque']))
        max_val = max(max(results['actual_torque']), max(results['predicted_torque']))
        plt.plot([min_val, max_val], [min_val, max_val], 'r--', label='Perfect Prediction')
        plt.xlabel('Actual Torque (Nm)')
        plt.ylabel('Predicted Torque (Nm)')
        plt.title('Predicted vs Actual Torque')
        plt.legend()
        plt.grid(True)
        plt.savefig(f"{base_save_path}_actual_vs_predicted.png", dpi=300)
        plt.show()

        # 2. Actual Torque vs Position Error
        plt.figure(figsize=(10, 8))
        plt.scatter(results['position_error'], results['actual_torque'], alpha=0.5, s=1, label='Actual')
        plt.xlabel('Position Error (rad)')
        plt.ylabel('Torque (Nm)')
        plt.title('Actual Torque vs Position Error')
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{base_save_path}_actual_vs_position.png", dpi=300)
        plt.show()

        # 3. Predicted Torque vs Position Error
        plt.figure(figsize=(10, 8))
        plt.scatter(results['position_error'], results['predicted_torque'], alpha=0.5, s=1, label='Predicted')
        plt.xlabel('Position Error (rad)')
        plt.ylabel('Torque (Nm)')
        plt.title('Predicted Torque vs Position Error')
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{base_save_path}_predicted_vs_position.png", dpi=300)
        plt.show()

        # 4. Actual Torque vs Velocity
        plt.figure(figsize=(10, 8))
        plt.scatter(results['velocity'], results['actual_torque'], alpha=0.5, s=1, label='Actual')
        plt.xlabel('Velocity (rad/s)')
        plt.ylabel('Torque (Nm)')
        plt.title('Actual Torque vs Velocity')
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{base_save_path}_actual_vs_velocity.png", dpi=300)
        plt.show()

        # 5. Predicted Torque vs Velocity
        plt.figure(figsize=(10, 8))
        plt.scatter(results['velocity'], results['predicted_torque'], alpha=0.5, s=1, label='Predicted')
        plt.xlabel('Velocity (rad/s)')
        plt.ylabel('Torque (Nm)')
        plt.title('Predicted Torque vs Velocity')
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{base_save_path}_predicted_vs_velocity.png", dpi=300)
        plt.show()

        # 6. Time series comparison
        plt.figure(figsize=(10, 8))
        samples = range(200)  # First 200 samples
        plt.plot(samples, results['actual_torque'][:200], label='Actual')
        plt.plot(samples, results['predicted_torque'][:200], label='Predicted')
        plt.xlabel('Sample')
        plt.ylabel('Torque (Nm)')
        plt.title('Torque Time Series Comparison')
        plt.legend()
        plt.grid(True)
        plt.savefig(f"{base_save_path}_time_series.png", dpi=300)
        plt.show()

        # Print denormalized statistics
        error = np.array(results['actual_torque']) - np.array(results['predicted_torque'])
        print("\nDenormalized Metrics:")
        print(f"Mean Error: {np.mean(error):.4f} Nm")
        print(f"RMSE: {np.sqrt(np.mean(error ** 2)):.4f} Nm")
        print(f"MAE: {np.mean(np.abs(error)):.4f} Nm")
        print(f"Max Error: {np.max(np.abs(error)):.4f} Nm")
        print(f"R²: {r2_score(results['actual_torque'], results['predicted_torque']):.4f}")

        # Additional statistics
        print("\nValue Ranges:")
        print(f"Position Error: [{min(results['position_error']):.4f}, {max(results['position_error']):.4f}] rad")
        print(f"Velocity: [{min(results['velocity']):.4f}, {max(results['velocity']):.4f}] rad/s")
        print(f"Actual Torque: [{min(results['actual_torque']):.4f}, {max(results['actual_torque']):.4f}] Nm")
        print(f"Predicted Torque: [{min(results['predicted_torque']):.4f}, {max(results['predicted_torque']):.4f}] Nm")

    def save_model(self, path: str, scaler_params: Dict) -> None:
        """Save model and scaling parameters

        Args:
            path: Path to save the model (without extension)
            scaler_params: Dictionary containing normalization parameters
        """
        # Save model state
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'scaler_params': scaler_params,
            'input_size': self.model.network[0].in_features,
            'hidden_size': self.model.network[0].out_features,
            'num_layers': len([layer for layer in self.model.network if isinstance(layer, nn.Linear)])
        }, f"{path}.pth")

        # Save normalization parameters separately for easier access
        import json
        with open(f"{path}_scaler.json", 'w') as f:
            json.dump(scaler_params, f, indent=4)

    def load_model(self, path: str) -> Tuple[Dict, Dict]:
        checkpoint = torch.load(f"{path}.pt")
        # Reconstruct model architecture
        self.model = ActuatorNet(
            input_size=checkpoint['input_size'],
            hidden_size=checkpoint['hidden_size'],
            num_layers=checkpoint['num_layers']
        )

        # Load model weights
        self.model.load_state_dict(checkpoint['model_state_dict'])

        return checkpoint['model_state_dict'], checkpoint['scaler_params']

# Example usage:
def main():
    # Create dataset
    dataset = ActuatorDataset("../data/data_full_200125_1914.csv")
    dataset.preprocess_data()
    dataset.normalize_data()
    dataset.save_csv("../data/processed_data.csv")

    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    print(f"Train data {train_dataset}")
    #
    # # Create dataloaders
    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=64)
    #
    # # Create and train model
    model = ActuatorNet()
    trainer = ActuatorTrainer(model)
    history = trainer.train(train_loader, val_loader)

    trainer.plot_training_history(history, save_path='../results/mlp/training_history.png')
    trainer.plot_prediction_vs_actual(val_loader, save_path='../results/mlp/prediction_vs_actual.png')

    trainer.denormalize_and_compare(val_loader, dataset, base_save_path="comprehensive_comparison")
    save_path = "../models/mlp_motor_model"
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    trainer.save_model(save_path, dataset.scaler_params)

    # Test loading
    # loaded_state_dict, loaded_scaler_params = trainer.load_model(save_path)

if __name__ == "__main__":
    main()