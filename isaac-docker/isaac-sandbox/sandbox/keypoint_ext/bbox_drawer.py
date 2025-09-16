
#!/usr/bin/env python3
import os, glob, cv2, numpy as np

INPUT_DIR = "./output"
OUTPUT_DIR = "./bbox_images"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def load_boxes_struct_npy(npy_path):
    arr = np.load(npy_path, allow_pickle=False)
    if arr.dtype.names is None:
        return []
    boxes = []
    for row in arr:
        x0, y0, x1, y1 = int(row["x_min"]), int(row["y_min"]), int(row["x_max"]), int(row["y_max"])
        if x1 > x0 and y1 > y0:
            boxes.append({
                "id": int(row["semanticId"]),
                "xyxy": [x0, y0, x1, y1],
                "occ": float(row["occlusionRatio"])
            })
    return boxes

def main():
    img_paths = sorted(glob.glob(os.path.join(INPUT_DIR, "rgb_*.png")))
    if not img_paths:
        print("No images found in", INPUT_DIR)
        return

    for img_path in img_paths:
        stem = os.path.splitext(os.path.basename(img_path))[0]   # rgb_0007
        idx = stem.split("_")[-1]                                # 0007
        npy_path = os.path.join(INPUT_DIR, f"bounding_box_2d_tight_{idx}.npy")
        if not os.path.exists(npy_path):
            print(f"[WARN] Missing {npy_path}")
            continue

        boxes = load_boxes_struct_npy(npy_path)
        if not boxes:
            print(f"[WARN] No boxes in {npy_path}")
            continue

        img = cv2.imread(img_path)
        overlay = img.copy()
        H, W = img.shape[:2]

        for b in boxes:
            x0, y0, x1, y1 = b["xyxy"]
            x0 = max(0, min(W-1, x0)); y0 = max(0, min(H-1, y0))
            x1 = max(0, min(W-1, x1)); y1 = max(0, min(H-1, y1))
            color = (0, 255, 0)
            cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)

            label = f"id:{b['id']} occ:{b['occ']:.2f}"
            ((tw, th), _) = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            bx0, by0 = x0, max(0, y0 - th - 6)
            cv2.rectangle(img, (bx0, by0), (bx0 + tw + 6, by0 + th + 6), color, -1)
            cv2.putText(img, label, (bx0 + 3, by0 + th + 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

        out_path = os.path.join(OUTPUT_DIR, f"{stem}_bbox.png")
        cv2.imwrite(out_path, img)
        print(f"[OK] wrote {out_path} ({len(boxes)} boxes)")

if __name__ == "__main__":
    main()