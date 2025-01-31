import itertools
import pandas as pd
import random

# Define battery colors
battery_colors = ['green', 'red', 'blue', 'orange', 'purple']

# Skeletons and variations for 2 combinations
skeletons_2 = [
    "Start at {start} and proceed to {final}.",
    "First, go to the {start} battery, then to the {final} battery.",
    "Begin at {start} and end at {final}.",
    "Go to {start} first, then proceed to {final}.",
    "Your task starts at {start} and finishes at {final}."
]
variations_2 = [
    "Make sure to start at {start} and then visit {final}.",
    "Head to {start} first, then to {final}.",
    "Visit {start}, and complete your task at {final}.",
    "The order is {start} followed by {final}.",
    "First stop is {start}, and the final stop is {final}."
]

# Skeletons and variations for 3 combinations
skeletons_3 = [
    "Start at {start}, then proceed to {next}, and end at {final}.",
    "Visit the {start} battery, make a stop at the {next} battery, and finish at the {final} battery.",
    "First, go to {start}, then head to {next}, and complete at {final}.",
    "Begin at {start}, continue to {next}, and wrap up at {final}.",
    "Your journey starts at {start}, proceeds through {next}, and ends at {final}."
]
variations_3 = [
    "Ensure you visit {start}, then {next}, and finally {final}.",
    "Go from {start} to {next}, and complete at {final}.",
    "Make stops at {start}, {next}, and finish at {final}.",
    "Start at {start}, continue to {next}, and end at {final}.",
    "Visit {start}, followed by {next}, and end your task at {final}."
]

# Skeletons and variations for 4 combinations
skeletons_4 = [
    "Start at {start}, continue to {next}, proceed to {middle}, and finish at {final}.",
    "Begin at {start}, make stops at {next} and {middle}, and complete at {final}.",
    "Your task starts at {start}, goes through {next} and {middle}, and ends at {final}.",
    "Visit {start}, then {next}, then {middle}, and finally {final}.",
    "First, go to {start}, proceed to {next} and {middle}, and wrap up at {final}."
]
variations_4 = [
    "Start at {start}, go through {next}, stop at {middle}, and end at {final}.",
    "Begin your journey at {start}, visit {next} and {middle}, and finish at {final}.",
    "Make stops at {start}, {next}, {middle}, and end at {final}.",
    "Go from {start} to {next}, through {middle}, and finish at {final}.",
    "Your route is {start}, {next}, {middle}, and {final}."
]

# Skeletons and variations for 5 combinations
skeletons_5 = [
    "Start at {start}, head to {next}, pass through {middle}, stop at {fourth}, and finish at {final}.",
    "Begin at {start}, then visit {next}, {middle}, {fourth}, and complete at {final}.",
    "Your task begins at {start}, continues to {next}, {middle}, {fourth}, and ends at {final}.",
    "Visit {start}, proceed to {next}, go through {middle}, stop at {fourth}, and end at {final}.",
    "First, go to {start}, then {next}, {middle}, and {fourth}, and complete at {final}."
]
variations_5 = [
    "Go from {start} to {next}, then to {middle}, {fourth}, and finally {final}.",
    "Make stops at {start}, {next}, {middle}, {fourth}, and end at {final}.",
    "Your journey starts at {start}, goes through {next}, {middle}, {fourth}, and ends at {final}.",
    "Start at {start}, proceed to {next}, pass {middle}, stop at {fourth}, and finish at {final}.",
    "Visit {start}, {next}, {middle}, {fourth}, and complete at {final}."
]

# Generate sentences with skeletons and variations
def generate_sentences(colors, skeletons, variations):
    data = []
    for combo in itertools.permutations(colors, len(skeletons[0].split("{")) - 1):
        for skeleton in skeletons:
            placeholders = {
                "start": combo[0],
                "next": combo[1] if len(combo) > 1 else "",
                "middle": combo[2] if len(combo) > 2 else "",
                "fourth": combo[3] if len(combo) > 3 else "",
                "final": combo[-1]
            }
            sentence = skeleton.format(**placeholders).strip()
            output = "; ".join(combo)
            data.append((sentence, output))
            for variation in variations:
                variation_sentence = variation.format(**placeholders).strip()
                data.append((variation_sentence, output))
    return data

# Generate data for all combinations
data_2 = generate_sentences(battery_colors, skeletons_2, variations_2)
data_3 = generate_sentences(battery_colors, skeletons_3, variations_3)
data_4 = generate_sentences(battery_colors, skeletons_4, variations_4)
data_5 = generate_sentences(battery_colors, skeletons_5, variations_5)

# Combine all data and shuffle
data_all = data_2 + data_3 + data_4 + data_5
random.shuffle(data_all)
data_all = data_all[:4781]  # Limit to 4781 sentences

# Convert to DataFrame
test_set_df = pd.DataFrame(data_all, columns=['Input_text', 'Output_text'])


import os

# Ensure the output directory exists
output_directory = 'Testing'  # Change this to a valid path as needed
os.makedirs(output_directory, exist_ok=True)

# Define the path for the output file
test_set_path = os.path.join(output_directory, 'navigation_dataset_Test.csv')

# Save the test set to a CSV file
test_set_df.to_csv(test_set_path, index=False)

print(f"Test set created and saved to {test_set_path}")
