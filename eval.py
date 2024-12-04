from transformers import AutoTokenizer, AutoModelForSeq2SeqLM, DataCollatorForSeq2Seq, Seq2SeqTrainingArguments, Seq2SeqTrainer
import torch
import numpy as np
from torch.utils.data import DataLoader, Dataset
import pandas as pd
import os
from tqdm import tqdm
import re

class TextToTextDataset(Dataset):
    def __init__(self, csv_file, tokenizer):
        self.data = pd.read_csv(csv_file)
        self.tokenizer = tokenizer

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        input_text = self.data.iloc[idx]['Input_text']
        output_text = self.data.iloc[idx]['Output_text']

        input_tokens = self.tokenizer(input_text, truncation=True, padding='max_length', max_length=128, return_tensors='pt')
        output_tokens = self.tokenizer(output_text, truncation=True, padding='max_length', max_length=128, return_tensors='pt')

        return {
            'input_ids': input_tokens['input_ids'].squeeze(),
            'attention_mask': input_tokens['attention_mask'].squeeze(),
            'labels': output_tokens['input_ids'].squeeze()
        }

model_path = "./trained_model/"
tokenizer = AutoTokenizer.from_pretrained(model_path)
model = AutoModelForSeq2SeqLM.from_pretrained(model_path)

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model.to(device)
model.eval()

eval_dataset = TextToTextDataset("./Validation/navigation_dataset_Validation.csv", tokenizer)
data_collator = DataCollatorForSeq2Seq(tokenizer=tokenizer, model=model)
dataloader = DataLoader(eval_dataset, batch_size=8, collate_fn=data_collator)

sequence_correct = 0
total_positions = 0
position_correct = 0
total_sequences = 0

with torch.no_grad():
    for batch in tqdm(dataloader):
        input_ids = batch['input_ids'].to(device)
        attention_mask = batch['attention_mask'].to(device)
        labels = batch['labels'].to(device)
        
        outputs = model.generate(
            input_ids=input_ids,
            attention_mask=attention_mask,
            max_length=128,
            pad_token_id=tokenizer.pad_token_id
        )

        predictions = [tokenizer.decode(output, skip_special_tokens=True) for output in outputs]
        true_labels = [tokenizer.decode(label[label != tokenizer.pad_token_id], skip_special_tokens=True) for label in labels]

        for pred, true in zip(predictions, true_labels):
            pred_colors = [color.strip() for color in re.split(r'[;,]', pred)]
            true_colors = [color.strip() for color in re.split(r'[;,]', true)]

            if pred_colors == true_colors:
                sequence_correct += 1
            total_sequences += 1

sequence_accuracy = sequence_correct / total_sequences

print("Accuracy: ",sequence_accuracy)
