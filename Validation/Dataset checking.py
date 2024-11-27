import torch
import pandas as pd

# Example function to generate output for a single input
def generate_output(input_text):
    # Move model to the same device as the input
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # Tokenize the input and move the inputs to the correct device
    inputs = tokenizer(input_text, return_tensors="pt", truncation=True, padding="max_length", max_length=128).to(device)
    
    # Make sure the model is also on the same device
    model.to(device)
    
    # Generate the output sequence
    output = model.generate(inputs['input_ids'], max_length=128)
    
    # Decode the generated tokens to text
    return tokenizer.decode(output[0], skip_special_tokens=True)

# Load the CSV file
input_csv_path = "navigation_dataset.csv"  # Replace with the path to your CSV file
output_csv_path = "navigation_dataset_with_outputs.csv"  # Output file to save generated results

# Read the input CSV file
data = pd.read_csv(input_csv_path)

# Ensure there's an 'instruction' column
if 'instruction' not in data.columns:
    raise ValueError("The input CSV must have an 'instruction' column.")

# Process each instruction and generate an output
generated_outputs = []
for instruction in data['instruction']:
    generated_output = generate_output(instruction)
    generated_outputs.append(generated_output)

# Add the generated outputs to the DataFrame
data['generated_output'] = generated_outputs

# Save the updated DataFrame to a new CSV file
data.to_csv(output_csv_path, index=False)

print(f"Processing complete. Output saved to {output_csv_path}")
