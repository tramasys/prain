import cv2, numpy as np, pytesseract

WHITELIST   = "ABC"
ANGLE_STEP  = 15
MIN_AREA    = 500
KERNEL      = np.ones((3,3), np.uint8)
TESS_CFG    = f"-c tessedit_char_whitelist={WHITELIST} --oem 3 --psm 10"

def _filter_letter(src_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(src_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, 1)

    h, w = mask.shape
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        touches = x == 0 or y == 0 or x+bw == w or y+bh == h
        if touches or area < MIN_AREA:
            mask[lbl == i] = 0

    return 255 - mask

def _ocr_letter(img_gray: np.ndarray) -> str | None:
    txt = pytesseract.image_to_string(img_gray, config=TESS_CFG).strip().upper()
    return txt[0] if txt and txt[0] in WHITELIST else None

def _rotate(img: np.ndarray, angle: int) -> np.ndarray:
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w/2, h/2), angle, 1.0)
    return cv2.warpAffine(img, m, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

def detect_letter(bgr: np.ndarray) -> tuple[str | None, int]:
    """
    bgr: full-resolution frame of the node.
    returns (letter or None, rotation_deg)
    """
    filtered = _filter_letter(bgr)
    letter   = _ocr_letter(filtered)
    if letter: return letter, 0

    for ang in range(ANGLE_STEP, 360, ANGLE_STEP):
        cand = _ocr_letter(_rotate(filtered, ang))
        if cand: return cand, ang

    return None, 0
