import itertools
import pandas as pd

# Define the list of colors
colors = ["green", "red", "orange", "purple", "blue"]

# Define a color-to-battery mapping
color_to_battery = {
    "green": "green",
    "red": "red",
    "orange": "orange",
    "purple": "purple",
    "blue": "blue"
}

# Read templates from the text file
with open("Skeleton - Validation.txt", "r", encoding="utf-8") as file:
    # Assume each line is a separate template
    templates = [line.strip() for line in file if line.strip()]

# Initialize a list to store the generated sentences along with their concatenated outputs
dataset = []

# Iterate over each template
for template in templates:
    # Determine the number of placeholders in the template (e.g., [battery1], [battery2], etc.)
    placeholders = [f"[battery{i}]" for i in range(1, 6) if f"[battery{i}]" in template]
    num_batteries = len(placeholders)

    # Generate all combinations of `num_batteries` colors
    color_combinations = list(itertools.combinations(colors, num_batteries))

    # For each combination of colors, generate all permutations
    for color_combo in color_combinations:
        color_permutations = list(itertools.permutations(color_combo))

        # Iterate over each permutation
        for combo in color_permutations:
            # Start with the original template
            sentence = template

            # Initialize a list to hold the output assignments
            outputs = []

            # Replace each battery placeholder with the corresponding color from the permutation
            for i, placeholder in enumerate(placeholders):
                color = combo[i]
                sentence = sentence.replace(placeholder, f"{color} battery")
                output = color_to_battery.get(color, f"Go to color{i + 1}")  # Fallback if color not in mapping
                outputs.append(output)

            # Concatenate all outputs into a single sentence separated by semicolons
            concatenated_output = "; ".join(outputs)

            # Create a data entry combining the sentence and its concatenated outputs
            data_entry = {
                "Input_text": sentence,
                "Output_text": concatenated_output
            }

            # Append the data entry to the dataset
            dataset.append(data_entry)

# Convert the dataset list into a pandas DataFrame
df = pd.DataFrame(dataset)

# Save the DataFrame to a CSV file for easy access and manipulation
csv_filename = "navigation_dataset - Validation.csv"
df.to_csv(csv_filename, index=False, encoding="utf-8-sig")

print(f"Dataset generated and saved as '{csv_filename}'")