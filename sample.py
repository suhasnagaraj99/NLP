import pandas as pd
import random

def sample_data(input_file, output_file, sample_size):
    df = pd.read_csv(input_file)

    sampled_df = df.sample(n=sample_size, random_state=42)

    sampled_df.to_csv(output_file, index=False)
    print(f"Randomly sampled {sample_size} data points saved to {output_file}.")

input_file = './navigation_dataset.csv'
output_file = './sampled_dataset.csv'
sample_size = 5000

sample_data(input_file, output_file, sample_size)