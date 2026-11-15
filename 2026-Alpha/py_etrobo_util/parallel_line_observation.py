"""Read-only tentative black-line image observation for a benchmark."""
import cv2
import numpy as np


def observe_black_line(frame, exclude_bbox=None):
    """Return image-space candidate; never claim calibrated floor position."""
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    roi = gray[h * 5 // 8:, :]
    mask = cv2.inRange(roi, 0, 50)
    if exclude_bbox is not None:
        x, y, bw, bh = exclude_bbox
        margin = 4
        x1, x2 = max(0, x-margin), min(w, x+bw+margin)
        y1, y2 = max(h*5//8, y-margin), min(h, y+bh+margin)
        if x1 < x2 and y1 < y2:
            mask[y1-h*5//8:y2-h*5//8, x1:x2] = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        x, y, bw, bh = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        if area >= 30 and bh >= 12 and bh >= bw * 0.35:
            candidates.append((area, x, y, bw, bh))
    if not candidates:
        return {"valid": False, "x_px": None, "area_px": 0, "image_width": w, "excluded_bbox": exclude_bbox}
    area, x, y, bw, bh = max(candidates)
    return {"valid": True, "x_px": x + bw / 2.0, "area_px": round(area, 1), "image_width": w, "bbox": [x, y + h * 5 // 8, bw, bh], "excluded_bbox": exclude_bbox}


def observe_lap_line_candidate(frame, exclude_bbox=None):
    """Find an unanchored candidate; reject comparably large competing shapes."""
    h,w=frame.shape[:2];top=h*5//8
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);mask=cv2.inRange(gray[top:,:],0,50)
    if exclude_bbox is not None:
        x,y,bw,bh=exclude_bbox;pad=4
        x1,x2=max(0,x-pad),min(w,x+bw+pad);y1,y2=max(top,y-pad),min(h,y+bh+pad)
        if x1<x2 and y1<y2:mask[y1-top:y2-top,x1:x2]=0
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    choices=[]
    for c in contours:
        x,y,bw,bh=cv2.boundingRect(c);area=cv2.contourArea(c)
        if area>=30 and bh>=12 and bh>=bw*.35:choices.append((area,x,y,bw,bh))
    choices.sort(reverse=True)
    result={'valid':False,'x_px':None,'image_width':w,'track_state':'CANDIDATE','track_reason':'no_candidate','excluded_bbox':exclude_bbox}
    if not choices:return result
    if len(choices)>1 and choices[1][0]>=.7*choices[0][0] and abs((choices[0][1]+choices[0][3]/2)-(choices[1][1]+choices[1][3]/2))>32:
        result['track_reason']='ambiguous_candidate';return result
    area,x,y,bw,bh=choices[0]
    result.update(valid=True,x_px=x+bw/2,area_px=round(area,1),bbox=[x,y+top,bw,bh],track_reason='unanchored')
    return result


def observe_lap_line_with_white_sides(frame, exclude_bbox=None):
    """Select an elongated LAP line with white floor visible at both edges."""
    height, width = frame.shape[:2]
    top = height * 5 // 8
    roi = frame[top:]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(gray, 0, 50)
    # The blue gate strip joins the black sections in the camera image.
    blue = cv2.inRange(hsv, (90, 95, 38), (132, 255, 200))
    line = cv2.bitwise_or(dark, blue)
    black = cv2.bitwise_and(dark, cv2.bitwise_not(blue))
    white = cv2.inRange(hsv, (0, 0, 145), (179, 60, 255))
    if exclude_bbox is not None:
        x, y, box_width, box_height = exclude_bbox
        x1, x2 = max(0, x - 4), min(width, x + box_width + 4)
        y1, y2 = max(top, y - 4), min(height, y + box_height + 4)
        if x1 < x2 and y1 < y2:
            line[y1 - top:y2 - top, x1:x2] = 0
            white[y1 - top:y2 - top, x1:x2] = 0

    result = {
        'valid': False, 'x_px': None, 'image_width': width, 'track_state': 'CANDIDATE',
        'track_reason': 'no_white_sided_candidate', 'excluded_bbox': exclude_bbox,
        'near_uv': None, 'far_uv': None, 'paired_white_rows': 0,
    }
    count, labels, stats, _ = cv2.connectedComponentsWithStats(line, connectivity=8)
    choices = []
    for label in range(1, count):
        x, y, box_width, box_height, area = (int(value) for value in stats[label])
        if (box_height < max(24, len(roi) // 2) or area < 30
                or x == 0 or x + box_width == width):
            continue
        component = labels[y:y + box_height, x:x + box_width] == label
        black_pixels = black[y:y + box_height, x:x + box_width]
        if np.count_nonzero(black_pixels[component]) < area * 0.12:
            continue
        points = _lap_white_sided_points(component, white, x, y)
        if len(points) >= 4 and points[-1][1] - points[0][1] >= len(roi) // 3:
            choices.append((len(points), area, x, y, box_width, box_height, points))
    if not choices:
        return result

    choices.sort(reverse=True)
    rows, area, x, y, box_width, box_height, points = choices[0]
    center = x + box_width / 2
    if any(other_rows >= rows - 1 and other_area >= 0.7 * area
           and abs(other_x + other_width / 2 - center) > 32
           for other_rows, other_area, other_x, _, other_width, _, _ in choices[1:]):
        result['track_reason'] = 'ambiguous_white_sided_candidate'
        return result
    far, near = points[0], points[-1]
    result.update(
        valid=True, x_px=center, area_px=area, bbox=[x, y + top, box_width, box_height],
        track_reason='white_sided', near_uv=[near[0], near[1] + top],
        far_uv=[far[0], far[1] + top], paired_white_rows=rows,
    )
    return result


def _lap_white_sided_points(component, white, x, y):
    points = []
    widths = []
    for row in np.linspace(2, len(component) - 3, 9, dtype=int):
        occupied = np.flatnonzero(component[row])
        if len(occupied) < 3:
            continue
        left = x + int(occupied[0])
        right = x + int(occupied[-1])
        span = right - left + 1
        if (left < 10 or right + 10 >= white.shape[1] or len(occupied) < 0.85 * span
                or span > white.shape[1] * 0.15):
            continue
        floor = white[y + row]
        # Leave a narrow gap at each edge for blur and JPEG ringing.
        if (np.count_nonzero(floor[left - 10:left - 2]) >= 6
                and np.count_nonzero(floor[right + 3:right + 11]) >= 6):
            points.append(((left + right) / 2, int(y + row)))
            widths.append(span)
    if len(points) < 4:
        return points

    far, near = points[0], points[-1]
    slope = (near[0] - far[0]) / (near[1] - far[1])
    tolerance = max(3.0, float(np.median(widths)) * 0.20)
    if any(abs(cx - (far[0] + slope * (cy - far[1]))) > tolerance for cx, cy in points[1:-1]):
        return []
    return points
