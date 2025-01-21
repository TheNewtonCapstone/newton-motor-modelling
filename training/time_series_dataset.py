import torch
import pandas as pd
from torch.utils.data import Dataset


class TimeSeriesDataset(Dataset):
    def __init__(self, csv_file, input_columns, target_columns, seq_length=10, column_transforms=None, device="cpu"):
        """
        Args:
            csv_file (str): Path to the CSV file.
            input_columns (list of str): Names of columns to be used as inputs.
            target_columns (list of str): Names of columns to be used as targets.
            seq_length (int): Number of time steps in each sequence.
            column_transforms (dict, optional): Dictionary defining new columns as transformations of existing ones.
                                                 Example: {"PositionError": ("TargetPosition", "Position", lambda x, y: x - y)}
            device (str, optional): Device to be used (default: "cpu").
        """
        # Load CSV file
        self.data = pd.read_csv(csv_file)

        # Apply column transformations
        if column_transforms:
            for new_col, (col1, col2, func) in column_transforms.items():
                self.data[new_col] = func(self.data[col1], self.data[col2])

        # Convert selected columns to tensors
        self.input_columns = input_columns
        self.target_columns = target_columns
        self.seq_length = seq_length

        self.inputs = torch.tensor(self.data[input_columns].values, dtype=torch.float32, device=device)
        self.targets = torch.tensor(self.data[target_columns].values, dtype=torch.float32, device=device)

    def __len__(self):
        """Returns the number of sequences available."""
        return len(self.inputs) - self.seq_length  # Because we form rolling windows

    def __getitem__(self, index):
        """Returns a sequence of inputs and corresponding targets."""
        X_seq = self.inputs[index: index + self.seq_length]  # Extract input sequence
        y_seq = self.targets[index: index + self.seq_length]  # Extract target sequence

        return X_seq, y_seq
