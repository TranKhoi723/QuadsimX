"""
quadsim.sam_obstacles
========================
NHAN DIEN VAT CAN BANG CLICK CHUOT dung MobileSAM (Segment Anything, ban
nhe cho CPU) - khac voi color_obstacles.py (doan theo mau, co the sai) va
osm_obstacles.py (can toa do GPS), phuong an nay hoat dong voi BAT KY anh
nao (so do ve tay, anh cu, anh khong co GPS) va cho ranh gioi CHINH XAC
THEO PIXEL THAT cua vat the, khong phai doan theo mau/nguong.

Nguyen tac hoat dong (Segment Anything nhu Meta cong bo, ban MobileSAM cua
ChaoningZhang - nhe hon SAM goc ~66 lan, do chinh xac gan tuong duong):
    1. set_image(): ma hoa TOAN BO anh 1 lan (buoc CHAM, vai giay tren CPU) 
       - CHI can lam lai khi doi anh moi.
    2. predict(): voi cac diem nguoi dung da click (diem "+" = thuoc vat
       can, diem "-" = KHONG thuoc, dung de sua sai khi mask lan sang vung
       khac), tra ve mask trong ~0.1-0.2s (nhanh, dung cho tuong tac).
    3. Mask nhi phan duoc chuyen thanh polygon bang OpenCV (findContours +
       approxPolyDP de rut gon so diem), ra dung dinh dang no_fly_zones
       CUNG VOI waypoint_editor.detect_no_fly_zones() / color_obstacles.py
       / osm_obstacles.py - cam thang duoc vao pathfinding.plan_path_pixels().

Cai dat (THEM vao requirements_gui.txt, KHONG bat buoc neu khong dung tinh
nang nay - toan bo phan con lai cua app van chay binh thuong neu thieu):
    pip install torch torchvision opencv-python-headless timm
    pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"

Tai checkpoint MobileSAM (~40MB, 1 LAN DUY NHAT, luu vao thu muc weights/):
    curl -L -o weights/mobile_sam.pt \\
        https://raw.githubusercontent.com/ChaoningZhang/MobileSAM/master/weights/mobile_sam.pt
"""

import os
import numpy as np

DEFAULT_CHECKPOINT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "weights", "mobile_sam.pt"
)
CHECKPOINT_URL = "https://raw.githubusercontent.com/ChaoningZhang/MobileSAM/master/weights/mobile_sam.pt"


def is_available():
    """Kiem tra da cai du thu vien (torch, mobile_sam, cv2) chua - GUI dung
    de AN/HIEN tuy chon SAM thay vi bao loi kho hieu khi thieu thu vien."""
    try:
        import torch  # noqa: F401
        import cv2  # noqa: F401
        from mobile_sam import sam_model_registry  # noqa: F401
        return True
    except ImportError:
        return False


def checkpoint_exists(checkpoint_path=None):
    path = checkpoint_path or DEFAULT_CHECKPOINT_PATH
    return os.path.isfile(path)


def download_checkpoint(checkpoint_path=None, timeout=120):
    """Tai checkpoint MobileSAM (~40MB) tu GitHub chinh thuc neu chua co.
    Chi can goi 1 lan duy nhat (checkpoint duoc luu lai tren dia)."""
    import requests
    path = checkpoint_path or DEFAULT_CHECKPOINT_PATH
    os.makedirs(os.path.dirname(path), exist_ok=True)
    resp = requests.get(CHECKPOINT_URL, timeout=timeout, stream=True)
    resp.raise_for_status()
    with open(path, "wb") as f:
        for chunk in resp.iter_content(chunk_size=1 << 20):
            f.write(chunk)
    return path


class SAMObstacleSegmenter:
    """
    Boc goi MobileSAM cho luong lam viec "click de khoanh vung vat can".

    Dung:
        seg = SAMObstacleSegmenter()              # load model 1 lan (~1s)
        seg.set_image(image_rgb)                   # ma hoa anh (~2-5s tren CPU,
                                                     # CHI can goi lai khi DOI anh)
        mask, score = seg.predict(points=[(190,140)], labels=[1])
        zone = seg.mask_to_zone(mask)               # -> dict polygon pixel
    """

    def __init__(self, checkpoint_path=None, model_type="vit_t", device=None):
        import torch
        from mobile_sam import sam_model_registry, SamPredictor

        checkpoint_path = checkpoint_path or DEFAULT_CHECKPOINT_PATH
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(
                f"Khong tim thay checkpoint MobileSAM tai '{checkpoint_path}'. "
                "Goi quadsim.sam_obstacles.download_checkpoint() truoc, hoac tai thu cong "
                f"tu {CHECKPOINT_URL}"
            )

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        sam = sam_model_registry[model_type](checkpoint=checkpoint_path)
        sam.to(device=self.device)
        sam.eval()
        self._predictor = SamPredictor(sam)
        self._image_key = None  # dung de biet anh hien tai da duoc set_image() chua

    def set_image(self, image_rgb, image_key=None):
        """Ma hoa anh (buoc cham). image_key (vd hash/ten file) dung de GUI
        biet co can goi lai ham nay khong - tranh ma hoa lai anh cu moi lan
        nguoi dung click."""
        self._predictor.set_image(np.asarray(image_rgb))
        self._image_key = image_key

    def is_image_ready(self, image_key):
        return self._image_key is not None and self._image_key == image_key

    def predict(self, points, labels, multimask_output=True):
        """
        points: list[(x,y)] toa do pixel diem nguoi dung da click.
        labels: list[int] cung do dai, 1 = diem THUOC vat can (foreground),
                0 = diem KHONG thuoc (dung de "tru" phan mask lan sang nham).

        Tra ve: (mask (H,W) bool - mask TOT NHAT theo diem so cua model, score float)
        """
        point_coords = np.array(points, dtype=float)
        point_labels = np.array(labels, dtype=int)
        masks, scores, _ = self._predictor.predict(
            point_coords=point_coords, point_labels=point_labels,
            multimask_output=multimask_output,
        )
        best_idx = int(np.argmax(scores))
        return masks[best_idx], float(scores[best_idx])

    @staticmethod
    def mask_to_zone(mask, simplify_epsilon_px=2.0):
        """
        Chuyen 1 mask nhi phan (H,W) bool thanh 1 (hoac nhieu, neu mask bi
        dut roi thanh nhieu vung) no_fly_zone dang polygon toa do pixel -
        CUNG DINH DANG voi cac ham detect_no_fly_zones* khac trong du an.

        Tra ve: list[dict] (thuong chi co 1 phan tu, co the nhieu neu mask
        khong lien thong).
        """
        import cv2

        mask_u8 = (mask.astype(np.uint8)) * 255
        contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        zones = []
        for contour in contours:
            if cv2.contourArea(contour) < 15:  # bo qua vung li ti (nhieu/artifact)
                continue
            epsilon = max(simplify_epsilon_px, 0.005 * cv2.arcLength(contour, True))
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) < 3:
                continue
            points_px = approx.reshape(-1, 2).astype(float).tolist()
            zones.append({"type": "polygon", "points_px": points_px, "source": "sam"})
        return zones
