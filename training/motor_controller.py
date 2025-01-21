import torch


class MotorController:
    def __init__(self, model, device="cpu"):
        self.model = model.to(device)
        self.device = device
        self.model.eval()  # Set model to evaluation mode

        # Internal state tracking
        self.current_position = 0.0  # Start at an initial position
        self.current_velocity = 0.0  # Start with zero velocity

    def predict_torque(self, target_position, dt=0.01):
        """
        Predicts motor torque based on target position while tracking velocity.

        Args:
            target_position (float): Desired position of the motor.
            dt (float): Time step to update velocity (default: 0.01s).

        Returns:
            float: Predicted torque.
        """
        # Compute position error
        position_error = target_position - self.current_position

        # Prepare input tensor (Shape: [1, 1, 2] -> batch=1, seq_len=1, features=2)
        input_tensor = torch.tensor([[position_error, self.current_velocity]], dtype=torch.float32).to(self.device)
        input_tensor = input_tensor.unsqueeze(0)  # Add batch dimension

        # Predict torque using the model
        with torch.no_grad():
            predicted_torque = self.model(input_tensor).item()

        # Update velocity and position (simplified physics model)
        self.current_velocity += predicted_torque * dt  # Acceleration effect
        self.current_position += self.current_velocity * dt  # Update position

        return predicted_torque
