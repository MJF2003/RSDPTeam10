#!/usr/bin/env python3
import argparse
from pathlib import Path

import pandas as pd
from sklearn.preprocessing import LabelEncoder

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, models
from PIL import Image
import joblib


class BinColorDataset(Dataset):
    def __init__(self, csv_file, root_dir, label_encoder, transform=None):
        self.df = pd.read_csv(csv_file)
        self.root_dir = Path(root_dir)
        self.le = label_encoder
        self.transform = transform

        # Filter to only rows whose files exist
        self.df = self.df[self.df["filename"].apply(lambda f: (self.root_dir / f).exists())]

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        row = self.df.iloc[idx]
        img_path = self.root_dir / row["filename"]
        img = Image.open(img_path).convert("RGB")
        if self.transform:
            img = self.transform(img)
        color_id = self.le.transform([row["color"]])[0]
        return img, torch.tensor(color_id, dtype=torch.long)


class BinColorNet(nn.Module):
    def __init__(self, n_colors):
        super().__init__()
        backbone = models.resnet18(weights="IMAGENET1K_V1")
        in_feat = backbone.fc.in_features
        backbone.fc = nn.Identity()
        self.backbone = backbone
        self.fc = nn.Linear(in_feat, n_colors)

    def forward(self, x):
        feat = self.backbone(x)
        out = self.fc(feat)
        return out


def train_epoch(model, loader, optimizer, loss_fn, device):
    model.train()
    total_loss = 0.0
    for imgs, labels in loader:
        imgs = imgs.to(device)
        labels = labels.to(device)
        optimizer.zero_grad()
        logits = model(imgs)
        loss = loss_fn(logits, labels)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
    return total_loss / max(1, len(loader))


def eval_epoch(model, loader, loss_fn, device):
    model.eval()
    total_loss = 0.0
    correct = 0
    n = 0
    with torch.no_grad():
        for imgs, labels in loader:
            imgs = imgs.to(device)
            labels = labels.to(device)
            logits = model(imgs)
            loss = loss_fn(logits, labels)
            total_loss += loss.item()
            preds = logits.argmax(1)
            correct += (preds == labels).sum().item()
            n += labels.size(0)
    acc = correct / max(1, n)
    return total_loss / max(1, len(loader)), acc


def main():
    ap = argparse.ArgumentParser(description="Train bin color classifier (ResNet18).")
    ap.add_argument("--train_root", required=True, help="Folder with training images")
    ap.add_argument("--val_root", required=True, help="Folder with validation images")
    ap.add_argument("--train_csv", required=True, help="CSV with columns filename,color for train")
    ap.add_argument("--val_csv", required=True, help="CSV with columns filename,color for val")
    ap.add_argument("--out", default="bin_color_clf_out", help="Output directory")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--bs", type=int, default=32)
    ap.add_argument("--lr", type=float, default=1e-3)
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[info] Using device: {device}")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load CSVs
    df_train = pd.read_csv(args.train_csv)
    df_val = pd.read_csv(args.val_csv)

    # Fit label encoder on training colors
    colors = sorted(df_train["color"].dropna().unique())
    print(f"[info] Colors: {colors}")
    le = LabelEncoder().fit(colors)

    # Transforms (light augmentations for robustness)
    train_tfm = transforms.Compose([
        transforms.Resize((256, 256)),
        transforms.RandomResizedCrop(224, scale=(0.8, 1.0), ratio=(0.9, 1.1)),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.ColorJitter(
            brightness=0.25,
            contrast=0.25,
            saturation=0.20,
            hue=0.02,
        ),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
    ])

    val_tfm = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
    ])

    train_ds = BinColorDataset(args.train_csv, args.train_root, le, train_tfm)
    val_ds = BinColorDataset(args.val_csv, args.val_root, le, val_tfm)

    train_dl = DataLoader(train_ds, batch_size=args.bs, shuffle=True, num_workers=4)
    val_dl = DataLoader(val_ds, batch_size=args.bs, shuffle=False, num_workers=4)

    model = BinColorNet(len(colors)).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.CrossEntropyLoss()

    best_acc = 0.0

    for epoch in range(1, args.epochs + 1):
        train_loss = train_epoch(model, train_dl, optimizer, loss_fn, device)
        val_loss, val_acc = eval_epoch(model, val_dl, loss_fn, device)

        print(
            f"Epoch {epoch}/{args.epochs} "
            f"train_loss={train_loss:.4f} "
            f"val_loss={val_loss:.4f} "
            f"val_acc={val_acc:.3f}"
        )

        # Save last
        torch.save(model.state_dict(), out_dir / "last.pt")

        # Save best
        if val_acc > best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), out_dir / "best.pt")

    # Save label encoder
    joblib.dump({"color_encoder": le}, out_dir / "label_encoder.pkl")
    print(f"[done] Saved model + encoder to {out_dir}")


if __name__ == "__main__":
    main()
