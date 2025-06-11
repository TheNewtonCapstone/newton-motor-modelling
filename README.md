# Actuator Motor Model

A PyTorch-based neural network implementation for modeling and predicting actuator motor torque based on position error and velocity history. This project implements both Multi-Layer Perceptron (MLP) and LSTM architectures for motor control applications.

## 🚀 Overview

This project provides a complete pipeline for:
- Data collection and preprocessing of actuator motor data
- Training neural networks to predict required torque
- Model evaluation and visualization
- Real-time motor control integration

The model uses historical position error and velocity data to predict the required torque output, enabling precise motor control applications.

## 📁 Project Structure

```
├── collection/           # Data collection utilities
│   ├── step.py          # Step response collection
│   ├── timer.py         # Timing utilities
│   └── velocity.py      # Velocity data collection
├── data/                # Dataset storage
│   ├── complete/        # Complete datasets
│   ├── incomplete/      # Partial datasets
│   └── processed/       # Preprocessed data
├── training/            # Model training scripts
│   ├── mlp.py          # Multi-Layer Perceptron implementation
│   ├── lstm.py         # LSTM model implementation
│   └── motor_controller.py # Motor control integration
├── models/             # Saved trained models
├── results/            # Training results and plots
│   ├── mlp/           # MLP results
│   └── lstm/          # LSTM results
└── utils/             # Utility functions
    └── plotter.py     # Visualization utilities
```

## 🔧 Installation

### Prerequisites
- Python 3.8+
- PyTorch
- CUDA-capable GPU (optional, but recommended)

### Dependencies
```bash
pip install torch torchvision
pip install pandas numpy matplotlib scikit-learn
pip install jupyter notebook  # For running notebooks
```

### Quick Setup
```bash
git clone <repository-url>
cd actuator-motor-model
pip install -r requirements.txt  # If available
```

## 📊 Data Format

The model expects CSV data with the following columns:
- `time`: Timestamp
- `target_position`: Desired position (radians)
- `position`: Actual position (radians)
- `velocity`: Angular velocity (rad/s)
- `torque`: Applied torque (Nm) - target variable

### Data Processing
The system automatically:
- Calculates position error (`target_position - position`)
- Downsamples data to target frequency (default: 100 Hz)
- Creates time-series windows with configurable history steps
- Normalizes features using z-score normalization

## 🤖 Model Architectures

### Multi-Layer Perceptron (MLP)
- **Input**: 6 features (3 position errors + 3 velocities from t, t-0.01, t-0.02)
- **Architecture**: Configurable hidden layers with Softsign activation
- **Output**: Single torque value
- **Features**: Xavier weight initialization, early stopping, learning rate scheduling

### LSTM (Long Short-Term Memory)
- **Input**: Sequential time-series data
- **Architecture**: Configurable LSTM layers with dropout
- **Output**: Torque prediction
- **Features**: Handles variable-length sequences, temporal dependencies

## 🚀 Usage

### Basic Training Pipeline

```python
from training.mlp import ActuatorDataset, ActuatorNet, ActuatorTrainer
from torch.utils.data import DataLoader
import torch

# 1. Load and preprocess data
dataset = ActuatorDataset("data/complete/data_full_sin.csv")
dataset.preprocess_data()
dataset.normalize_data()

# 2. Create train/validation split
train_size = int(0.8 * len(dataset))
val_size = len(dataset) - train_size
train_dataset, val_dataset = torch.utils.data.random_split(
    dataset, [train_size, val_size]
)

# 3. Create data loaders
train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=64)

# 4. Initialize model and trainer
model = ActuatorNet(input_size=6, hidden_size=32, num_layers=3)
trainer = ActuatorTrainer(model)

# 5. Train the model
history = trainer.train(train_loader, val_loader, epochs=100)

# 6. Save the model
trainer.save_model("models/my_model", dataset.scaler_params)
```

### Data Collection

```python
from collection.step import collect_step_response
from collection.velocity import collect_velocity_data

# Collect step response data
collect_step_response(duration=60, output_file="data/step_response.csv")

# Collect velocity tracking data
collect_velocity_data(duration=120, output_file="data/velocity_tracking.csv")
```

### Model Evaluation

```python
# Generate comprehensive analysis
trainer.denormalize_and_compare(
    val_loader, 
    dataset, 
    base_save_path="results/analysis"
)

# Plot training history
trainer.plot_training_history(history, save_path="results/training.png")

# Prediction vs actual scatter plot
trainer.plot_prediction_vs_actual(val_loader, save_path="results/predictions.png")
```
### Key Parameters

```python
# Data preprocessing
HISTORY_STEPS = 3        # Number of time steps to use as input
TARGET_FREQUENCY = 100   # Target sampling frequency (Hz)

# Model architecture
INPUT_SIZE = 6          # 3 position errors + 3 velocities
HIDDEN_SIZE = 32        # Hidden layer size
NUM_LAYERS = 3          # Number of hidden layers

# Training parameters
LEARNING_RATE = 1e-3    # Initial learning rate
BATCH_SIZE = 64         # Training batch size
EPOCHS = 100            # Maximum training epochs
PATIENCE = 10           # Early stopping patience
```

### Hardware Requirements
- **CPU**: Multi-core processor recommended
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: CUDA-compatible GPU optional but recommended for faster training
- **Storage**: 1GB for datasets and models

## 📁 File Descriptions

### Core Files
- `training/mlp.py`: Complete MLP implementation with training pipeline
- `training/lstm.py`: LSTM model implementation
- `training/motor_controller.py`: Real-time motor control integration

### Data Collection
- `collection/step.py`: Step response data collection
- `collection/velocity.py`: Velocity tracking data collection
- `collection/timer.py`: Timing utilities for data collection

### Utilities
- `utils/plotter.py`: Visualization and plotting utilities
- `training/time_series_dataset.py`: Time series data handling

## 🎯 Applications

This model is designed for:
- **Robotic actuators**: Precise position and velocity control
- **Industrial automation**: Motor control in manufacturing
- **Research**: Motor dynamics modeling and analysis
- **Education**: Understanding neural networks in control systems

## 📊 Results Interpretation

### Training Outputs
- **Loss curves**: Monitor for overfitting and convergence
- **R² score**: Higher values (closer to 1.0) indicate better fit
- **RMSE/MAE**: Lower values indicate better accuracy
- **Residual plots**: Check for systematic errors

### Model Selection
- Compare MLP vs LSTM performance on your specific dataset
- Consider computational requirements for real-time applications
- Evaluate generalization to new operating conditions

---

**Note**: This project uses PyTorch and requires proper CUDA setup for GPU acceleration. Ensure your environment is properly configured before training large models.
