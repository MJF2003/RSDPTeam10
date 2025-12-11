# /home/laptop30/yolov5/clf/train_multitask.py
import argparse
from pathlib import Path
import pandas as pd
from sklearn.preprocessing import LabelEncoder
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, models
from PIL import Image

# ---------- Dataset ----------
class BlockDataset(Dataset):
    def __init__(self, csv_file, root_dir, color_enc, shape_enc, transform=None):
        self.df = pd.read_csv(csv_file)
        self.root_dir = Path(root_dir)
        self.color_enc = color_enc
        self.shape_enc = shape_enc
        self.transform = transform

        # filter missing files
        self.df = self.df[self.df["filename"].apply(lambda f: (self.root_dir / f).exists())]

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        row = self.df.iloc[idx]
        img_path = self.root_dir / row["filename"]
        img = Image.open(img_path).convert("RGB")
        if self.transform:
            img = self.transform(img)
        color_id = self.color_enc.transform([row["color"]])[0]
        shape_id = self.shape_enc.transform([row["shape"]])[0]
        return img, torch.tensor(color_id), torch.tensor(shape_id)

# ---------- Model ----------
class MultiTaskNet(nn.Module):
    def __init__(self, n_colors, n_shapes):
        super().__init__()
        backbone = models.resnet18(weights="IMAGENET1K_V1")
        in_feat = backbone.fc.in_features
        backbone.fc = nn.Identity()
        self.backbone = backbone
        self.fc_color = nn.Linear(in_feat, n_colors)
        self.fc_shape = nn.Linear(in_feat, n_shapes)

    def forward(self, x):
        feat = self.backbone(x)
        return self.fc_color(feat), self.fc_shape(feat)

# ---------- Training ----------
def train_epoch(model, loader, opt, loss_fn, device):
    model.train()
    tot_loss = 0
    for imgs, col, shp in loader:
        imgs, col, shp = imgs.to(device), col.to(device), shp.to(device)
        opt.zero_grad()
        out_col, out_shp = model(imgs)
        loss = loss_fn(out_col, col) + loss_fn(out_shp, shp)
        loss.backward()
        opt.step()
        tot_loss += loss.item()
    return tot_loss / len(loader)

def val_epoch(model, loader, loss_fn, device):
    model.eval()
    tot_loss = 0
    c_corr, s_corr, n = 0, 0, 0
    with torch.no_grad():
        for imgs, col, shp in loader:
            imgs, col, shp = imgs.to(device), col.to(device), shp.to(device)
            out_col, out_shp = model(imgs)
            loss = loss_fn(out_col, col) + loss_fn(out_shp, shp)
            tot_loss += loss.item()
            c_corr += (out_col.argmax(1) == col).sum().item()
            s_corr += (out_shp.argmax(1) == shp).sum().item()
            n += len(col)
    return tot_loss / len(loader), c_corr / n, s_corr / n

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train_root", required=True)
    ap.add_argument("--val_root", required=True)
    ap.add_argument("--train_csv", required=True)
    ap.add_argument("--val_csv", required=True)
    ap.add_argument("--out", default="clf_out")
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--bs", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Using", device)

    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)

    # Load CSVs
    df_train = pd.read_csv(args.train_csv)
    df_val   = pd.read_csv(args.val_csv)
    colors = sorted(df_train["color"].dropna().unique())
    shapes = sorted(df_train["shape"].dropna().unique())
    color_enc = LabelEncoder().fit(colors)
    shape_enc = LabelEncoder().fit(shapes)

    print(f"Classes: colors={colors}, shapes={shapes}")

    # Datasets & loaders
    tfm = transforms.Compose([
        transforms.Resize((224,224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
    ])
    train_ds = BlockDataset(args.train_csv, args.train_root, color_enc, shape_enc, tfm)
    val_ds   = BlockDataset(args.val_csv, args.val_root, color_enc, shape_enc, tfm)
    train_dl = DataLoader(train_ds, batch_size=args.bs, shuffle=True, num_workers=4)
    val_dl   = DataLoader(val_ds, batch_size=args.bs, shuffle=False, num_workers=4)

    model = MultiTaskNet(len(colors), len(shapes)).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.CrossEntropyLoss()

    best_acc = 0
    for epoch in range(args.epochs):
        tr_loss = train_epoch(model, train_dl, opt, loss_fn, device)
        val_loss, c_acc, s_acc = val_epoch(model, val_dl, loss_fn, device)
        print(f"Epoch {epoch+1}/{args.epochs} "
              f"train_loss={tr_loss:.4f} val_loss={val_loss:.4f} "
              f"color_acc={c_acc:.3f} shape_acc={s_acc:.3f}")
        torch.save(model.state_dict(), out_dir / "last.pt")
        if (c_acc + s_acc)/2 > best_acc:
            best_acc = (c_acc + s_acc)/2
            torch.save(model.state_dict(), out_dir / "best.pt")

    # save label encoders
    import joblib
    joblib.dump({"color_enc": color_enc, "shape_enc": shape_enc},
                out_dir / "label_encoders.pkl")
    print(f"[done] Saved model and encoders to {out_dir}")

if __name__ == "__main__":
    main()
