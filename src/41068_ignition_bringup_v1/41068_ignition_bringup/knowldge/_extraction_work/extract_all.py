#!/usr/bin/env python3
"""Multi-pass PDF extraction pipeline for PowerPoint slide decks."""

import fitz
import json
import os
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime

PDF_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK_DIR = os.path.dirname(os.path.abspath(__file__))
PAGES_DIR = os.path.join(WORK_DIR, "pages")
IMAGES_DIR = os.path.join(WORK_DIR, "images")
OCR_DIR = os.path.join(WORK_DIR, "ocr")

PDFS = [
    "decision_making.pdf",
    "path_planning.pdf",
    "perception_mapping.pdf",
    "simulation_package.pdf",
]

ZOOM = 2.0  # render resolution multiplier


def ensure_dirs():
    for d in [PAGES_DIR, IMAGES_DIR, OCR_DIR]:
        os.makedirs(d, exist_ok=True)


def extract_slide_text(page):
    """Extract structured text blocks from a slide page."""
    blocks = page.get_text("dict")["blocks"]
    text_spans = []
    images = []

    for b in blocks:
        if b["type"] == 0:
            for line in b["lines"]:
                for span in line["spans"]:
                    t = span["text"].strip()
                    if t:
                        text_spans.append({
                            "text": t,
                            "size": round(span["size"], 1),
                            "font": span["font"],
                            "bbox": list(span["bbox"]),
                            "y": span["bbox"][1],
                            "x": span["bbox"][0],
                            "bold": bool(span["flags"] & 2**4),
                        })
        elif b["type"] == 1:
            images.append({
                "bbox": list(b["bbox"]),
                "width": b.get("width"),
                "height": b.get("height"),
            })

    # Sort by y then x for reading order
    text_spans.sort(key=lambda s: (round(s["y"] / 10) * 10, s["x"]))

    # Identify title (largest font, excluding footer patterns)
    footer_patterns = re.compile(
        r"^(41068 Robotics Studio 1|\d+)$", re.IGNORECASE
    )
    content_spans = [
        s for s in text_spans if not footer_patterns.match(s["text"])
    ]

    title = ""
    if content_spans:
        max_size = max(s["size"] for s in content_spans)
        title_parts = [
            s["text"]
            for s in content_spans
            if s["size"] >= max_size * 0.9 and s["y"] < page.rect.height * 0.4
        ]
        title = " ".join(title_parts).strip()

    # Group body text (exclude title and footer)
    body = []
    slide_num = None
    for s in text_spans:
        if re.match(r"^\d+$", s["text"]) and s["y"] < 50:
            slide_num = s["text"]
            continue
        if s["text"] == "41068 Robotics Studio 1":
            continue
        if title and s["text"] in title and s["size"] >= max_size * 0.9:
            continue
        body.append(s["text"])

    return {
        "title": title,
        "slide_num": slide_num,
        "body_lines": body,
        "all_spans": text_spans,
        "embedded_images": images,
        "image_count": len(images),
    }


def ocr_page(page_path):
    """OCR a rendered page image using tesseract."""
    try:
        result = subprocess.run(
            ["tesseract", page_path, "stdout", "--psm", "6"],
            capture_output=True,
            text=True,
            timeout=60,
        )
        return result.stdout.strip()
    except Exception as e:
        return f"[OCR failed: {e}]"


def extract_embedded_images(doc, pdf_name, page_idx):
    """Extract embedded images from a page."""
    page = doc[page_idx]
    img_list = page.get_images(full=True)
    extracted = []
    pdf_stem = pdf_name.replace(".pdf", "")

    for img_idx, img in enumerate(img_list):
        xref = img[0]
        try:
            base = doc.extract_image(xref)
            ext = base["ext"]
            img_bytes = base["image"]
            fname = f"{pdf_stem}_p{page_idx+1:03d}_img{img_idx+1:02d}.{ext}"
            fpath = os.path.join(IMAGES_DIR, fname)
            if not os.path.exists(fpath):
                with open(fpath, "wb") as f:
                    f.write(img_bytes)
            extracted.append({
                "file": fname,
                "path": fpath,
                "width": base.get("width"),
                "height": base.get("height"),
                "colorspace": base.get("colorspace"),
            })
        except Exception:
            pass
    return extracted


def classify_slide(slide_data, ocr_text):
    """Classify slide type for extraction strategy."""
    text_len = len(" ".join(slide_data["body_lines"]))
    img_count = slide_data["image_count"]
    if text_len < 30 and img_count >= 2:
        return "visual_heavy"
    if text_len < 80 and img_count >= 1:
        return "mixed"
    return "text"


def process_pdf(pdf_name):
    """Full multi-pass extraction for one PDF."""
    path = os.path.join(PDF_DIR, pdf_name)
    doc = fitz.open(path)
    meta = doc.metadata
    pdf_stem = pdf_name.replace(".pdf", "")
    slides = []

    print(f"Processing {pdf_name} ({len(doc)} pages)...")

    for i in range(len(doc)):
        page = doc[i]
        slide_data = extract_slide_text(page)

        # Render page for OCR
        page_img_path = os.path.join(PAGES_DIR, f"{pdf_stem}_p{i+1:03d}.png")
        if not os.path.exists(page_img_path):
            mat = fitz.Matrix(ZOOM, ZOOM)
            pix = page.get_pixmap(matrix=mat, alpha=False)
            pix.save(page_img_path)

        # OCR
        ocr_path = os.path.join(OCR_DIR, f"{pdf_stem}_p{i+1:03d}.txt")
        if not os.path.exists(ocr_path):
            ocr_text = ocr_page(page_img_path)
            with open(ocr_path, "w", encoding="utf-8") as f:
                f.write(ocr_text)
        else:
            with open(ocr_path, "r", encoding="utf-8") as f:
                ocr_text = f.read()

        # Extract embedded images
        embedded = extract_embedded_images(doc, pdf_name, i)

        slide_type = classify_slide(slide_data, ocr_text)

        slides.append({
            "page": i + 1,
            "title": slide_data["title"],
            "body_lines": slide_data["body_lines"],
            "all_text_native": "\n".join(
                s["text"] for s in slide_data["all_spans"]
            ),
            "ocr_text": ocr_text,
            "slide_type": slide_type,
            "embedded_images": embedded,
            "image_count": slide_data["image_count"],
            "page_render": os.path.basename(page_img_path),
        })

        if (i + 1) % 10 == 0:
            print(f"  {i+1}/{len(doc)} pages done")

    doc.close()

    result = {
        "source_file": pdf_name,
        "metadata": meta,
        "page_count": len(slides),
        "extraction_date": datetime.now().isoformat(),
        "slides": slides,
    }

    out_json = os.path.join(WORK_DIR, f"{pdf_stem}_extracted.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print(f"  Saved {out_json}")
    return result


def main():
    ensure_dirs()
    all_results = {}
    for pdf in PDFS:
        all_results[pdf] = process_pdf(pdf)
    print("\nExtraction complete.")
    return all_results


if __name__ == "__main__":
    main()
