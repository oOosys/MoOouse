#!/usr/bin/python3
#							oOosys (ChatGPT)						     2025-10-16_01:05
#																	     2025-10-17_06:54
minSnakeAccumulativeLength	=   80.0
maxSnakeAccumulativeLength	= 520.0
collisionThreshold				=    8.0
lineThreshold					=    0.3	# if inverted roundness < lineThreshold 
# ^-- above this threshold -> emit the nibble directly as codepoint (line-like slim contour)
canvasWidth, canvasHeight		= 1600, 900
snakeSegmentPointRadius		= 11
oOoInfo = """\
	This DEMO application allows to test a mouse motion only based input to a text edit 
		area of an PyQt6 based GUI.  The mouse pointer is tracked continuously and the 
			screen shows in green color all the pixel the mouse pointer visited in the past
				which satisfy the condition of being less distant from last position  than
					 minSnakeAccumulativeLength. 
	The shown path of the mouse is considered to be a "snake" and all the mouse path
		points with accumulative distance to the current point (the head of the snake)
			less than maxSnakeAccumulativeLength  appear cyan. 
	A closed mouse motion path  is detected when the "head" point of the snake "bumps" 
		into its cyan tail "baiting itself". The detected closed contour is then analysed 
			and the results are shown in the header line and are visualized on the canvas 
				The snake re-starts then at zero length its "next life" after the "self-bite". 
	How the closed contour is analysed? It is determined if it is CW or CCW and from the 
		start/end point cumulative distances are calculated and memorized for later use 
			going from start towards the end and at the same time going from end to 
				the start. This procedure ends at a point which distance from the start 
					and from the end is nearly the same (within a tolerance). 
	Knowing the entire cumulative length of the snake two more points are 
		determined: they are at 1/4 length of the snake : one from its head to tail, one 
			from the tail towards head. 
	Knowing the points at 1/4, 1/2 and 3/4 of the snake cumulative length  the "roundness"
		 of the contour can be determined as a ratio of lengths of two line sections : 
			one connecting start/end point with the point at 1/2 snake length and the 
				others connecting the points at 1/4 snake length. 
	Roundness near 1.0 suggests a circle contour, roundness lower than one suggests an 
		ellipsis like shape with width (the length of line between 1/4 snake length points) 
			lower than the length. Ratio greater 1 suggests an "overwheight" like contour 
				shape ( :D LOL ). 
	Along with this ratio the CW/CCW of the contour, the total contour length, the ratio 
		of roundness and the angle of the vector between the start/end ( head/tail of the 
			snake) and the point at 1/2 length of the contour are reported. 
	At this time the "snake" length resets to zero and the next round until the snake bites 
		own tail starts. 
	Each closed contour after the snake bites its tail delivers CW/CCW orientation of the 
		mouse motion path and the angle which when "digitized" into 8 main compass 
			directions give together 16 different states.
	 Pairs of these closed contours give this way all ASCII values from 0 to 255 and can be 
		used to as Unicode code point values of a character. 
			To make it possible to enter Tab, Backspace, Return with one "bite" the 
				roundness factor is analysed ... if the shape is very slim (almost a line, but 
					still "round" enough to unambiguously determine CW/CCW) the code 
						of the first nibble is already the Unicode code point to emit to the 
							text edit area. 
	A status header line displays all the calculated values : CW/CCW, the "digitized" angle, 
		the roundness ratio, the nibble hex code and if a pair of nibbles form the Unicode 
			code point the hex code of the code point. 
	The GUI with the snake should show the points on the contour and the lines used to 
		determine the roundness ratio and the angle. The overall closed contour length is 
			also shown in the header status line. The text edit area resides below the 
				mouse motion tracking area/canvas. 
	The application is named OApp and its window OAppO . It is written in a flat style 
		without defining classes or 'if __name__== ...'. 
			Tabs are used for indentation, CamelCase for names of variables which should 
				escribe the outcome of what they are or do (i.e. the new state after a 
					function run or the meaning of a value).  
	Looong symbol names are fully ok ... they should make commenting and documentation 
		bsolete due to their intuitive expressiveness allowing to read the code like a text 
			in English language. The first nibble of the pair should be the low nibble, 
				the second one the high nibble of the byte representing the Unicode 
					code point. 
	Along with the code there are instructions how to "type" a-z A-Z 0-9, [{}] () !@#$%^&*. 
		This code uses the "clock convention" where the ZERO (midnight hour) is at 90 
			degree at the top. The 45 degree CW are then 1, 90 degree is 2 and 
				315 degree is  7, CW means 0-7, CCW means 8-15. 
	THIS description is sufficient for an AI to provide appropriate code and is therefore
		the actual program code ...
"""


# gesture_keyboard_oOo_flat.py
# Flat procedural PyQt6 app in oOo naming: OApp (app), OAppO (window)
# oOo-style: outcome-first names, camelCase, Tabs for indentation.

from PyQt6 import QtWidgets, QtGui, QtCore
import sys
OApp = QtWidgets.QApplication(sys.argv)
from PyQt6.QtGui import QImage
backgroundImageFileName = f"{__file__[:-6]}_canvasBackground_o0.png"
backgroundQImage = QImage(backgroundImageFileName)
#print(f"{backgroundImageFileName=}  , {backgroundQImage=}")
#print(f"{backgroundQImage.isNull()=}")

# ---------- oOo global state ----------
OAppO = QtWidgets.QWidget()
OAppO.setWindowTitle("oOo Gesture Keyboard (flat) - OAppO")
OAppO.resize(1000,760)

SnakePoints = []    # head at index 0
BiteBuffer = None   # waiting high nibble (0..15) or None
analysisResult = None
lastGestureInfo = None
recording = False   # left-click starts recording; right-click or self-bite resets

import math
# ---------- geometry helpers ----------
def Dist_between_Points(a, b):
	(ax,ay) = a; (bx,by) = b
	dx = bx-ax; dy = by-ay
	return math.hypot(dx, dy)

def InterpolatePoint(a, b, t):
	return (a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t)

def AreaPolygon_signed_from_PointCloud(points):
	if len(points) < 3:
		return 0.0
	s = 0.0
	for i in range(len(points)):
		x1,y1 = points[i]
		x2,y2 = points[(i+1)%len(points)]
		s += x1*y2 - x2*y1
	return 0.5 * s

def PointAlongPolyline_from_Start(points, targetDist):
	if targetDist <= 0.0:
		return points[0]
	acc = 0.0
	for i in range(len(points)-1):
		a = points[i]; b = points[i+1]
		seg = Dist_between_Points(a,b)
		if acc + seg >= targetDist:
			rem = targetDist - acc
			t = 0.0 if seg == 0.0 else (rem/seg)
			return InterpolatePoint(a, b, t)
		acc += seg
	return points[-1]

def CumulativeDistances_from_Head(points):
	n = len(points)
	if n == 0:
		return []
	dists = [0.0]*n
	acc = 0.0
	for i in range(1,n):
		acc += Dist_between_Points(points[i-1], points[i])
		dists[i] = acc
	return dists

# ---------- analysis (inverted roundness) ----------
def AnalyzeClosedContour_from_SnakeSegment(pointsSegment):
	# returns dict with orientation, totalLength, halfPoint, quarterFromHead, quarterFromTail,
	# roundnessInv (denom/numer), angleDeg_head_to_half, contourPolygon
	n = len(pointsSegment)
	if n < 2:
		raise ValueError("contour too small")
	polygon = pointsSegment[:]
	areaSigned = AreaPolygon_signed_from_PointCloud(polygon + [polygon[0]])
	orientation = "CCW" if areaSigned > 0 else "CW"
	cum = CumulativeDistances_from_Head(polygon)
	totalLen = cum[-1]
	if totalLen <= 1e-9:
		halfPoint = polygon[0]
		qHead = polygon[0]
		qTail = polygon[-1]
	else:
		halfPoint = PointAlongPolyline_from_Start(polygon, totalLen * 0.5)
		qHead = PointAlongPolyline_from_Start(polygon, totalLen * 0.25)
		qTail = PointAlongPolyline_from_Start(polygon, totalLen * 0.75)
	head = polygon[0]
	numer = Dist_between_Points(head, halfPoint)          # original numerator
	denom = Dist_between_Points(qHead, qTail)            # original denominator
	if numer == 0.0:
		roundnessInv = float('inf')
	else:
		roundnessInv = denom / numer
	vx = halfPoint[0] - head[0]; vy = halfPoint[1] - head[1]
	angleDeg = -112.7 + (math.degrees(math.atan2(vy, vx)) % 360.0) if (abs(vx) > 1e-12 or abs(vy) > 1e-12) else 0.0
	return {
		'orientation': orientation,
		'totalLength': totalLen,
		'halfPoint': halfPoint,
		'quarterFromHead': qHead,
		'quarterFromTail': qTail,
		'roundnessInv': roundnessInv,
		'angleDeg_head_to_half': angleDeg,
		'contourPolygon': polygon
	}

# ---------- nibble logic ----------
def AngleBin_from_AngleDeg(angleDeg):
	binIdx = ( 5 + int((angleDeg % 360.0) // 45.0) ) % 8
	return binIdx

def Nibble_from_Analysis(analysis):
	angleBin = AngleBin_from_AngleDeg(analysis['angleDeg_head_to_half'])
	rotationBit = 0 if analysis['orientation'] == "CW" else 1
	nibble = angleBin + (8 if rotationBit == 1 else 0)
	return nibble, angleBin, rotationBit

# ---------- UI layout ----------
canvas = QtWidgets.QWidget(OAppO)
canvas.setFixedHeight(canvasHeight)
canvas.setFixedWidth(canvasWidth)
canvas.setMouseTracking(True)
textEdit = QtWidgets.QTextEdit(OAppO)
textEdit.setPlainText("")
textEdit.setReadOnly(False)

layout = QtWidgets.QVBoxLayout(OAppO)
layout.setContentsMargins(4,4,4,4)
layout.setSpacing(6)
layout.addWidget(canvas)
layout.addWidget(textEdit)

# ---------- drawing ----------
def canvas_paintEvent(event):

	p = QtGui.QPainter(canvas)
	p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
	p.fillRect(canvas.rect(), QtGui.QColor(0,100,0))
	p.drawImage(0, 0, backgroundQImage)

	# draw snake stamps
	if len(SnakePoints) > 0:
		cum = CumulativeDistances_from_Head(SnakePoints)
		greenLimitIndex = len(SnakePoints)-1
		for idx,d in enumerate(cum):
			if d >= minSnakeAccumulativeLength:
				greenLimitIndex = idx
				break
		for i, pt in enumerate(SnakePoints):
			x,y = pt
			if i < greenLimitIndex:
				color = QtGui.QColor(255,0,0); r = snakeSegmentPointRadius - 2
			else:
				color = QtGui.QColor(0,255,0); r = snakeSegmentPointRadius
			p.setBrush(QtGui.QBrush(color))
			p.setPen(QtGui.QPen(QtGui.QColor(0,0,0,150)))
			p.drawEllipse(QtCore.QPointF(x,y), r, r)

	# overlay of last analysis if present
	if analysisResult is not None:
		res = analysisResult
		# polygon
		p.setPen(QtGui.QPen(QtGui.QColor(255,255,200), 2))
		poly = QtGui.QPolygonF([QtCore.QPointF(x,y) for (x,y) in res['contourPolygon']])
		if len(poly) > 1:
			p.drawPolyline(poly)
		# markers
		head = res['contourPolygon'][0]
		tail = res['contourPolygon'][-1]
		half = res['halfPoint']
		qh = res['quarterFromHead']
		qt = res['quarterFromTail']
		p.setBrush(QtGui.QBrush(QtGui.QColor(255,200,0))); p.drawEllipse(QtCore.QPointF(head[0], head[1]), 6, 6)
		p.setBrush(QtGui.QBrush(QtGui.QColor(200,0,200))); p.drawEllipse(QtCore.QPointF(tail[0], tail[1]), 6, 6)
		p.setBrush(QtGui.QBrush(QtGui.QColor(255,255,255))); p.drawEllipse(QtCore.QPointF(half[0], half[1]), 5, 5)
		p.setBrush(QtGui.QBrush(QtGui.QColor(255,140,0))); p.drawEllipse(QtCore.QPointF(qh[0], qh[1]), 5, 5); p.drawEllipse(QtCore.QPointF(qt[0], qt[1]), 5, 5)
		# ratio lines
		p.setPen(QtGui.QPen(QtGui.QColor(220,220,255), 2, QtCore.Qt.PenStyle.DashLine))
		p.drawLine(QtCore.QPointF(head[0],head[1]), QtCore.QPointF(half[0],half[1]))
		p.setPen(QtGui.QPen(QtGui.QColor(200,255,200), 2, QtCore.Qt.PenStyle.SolidLine))
		p.drawLine(QtCore.QPointF(qh[0],qh[1]), QtCore.QPointF(qt[0],qt[1]))

	# header text
	p.setPen(QtGui.QPen(QtGui.QColor(230,230,230)))
	p.setFont(QtGui.QFont("Sans", 16))
	header = buildHeaderText()
	p.drawText(8, 28, header)

	# footer
	p.setPen(QtGui.QPen(QtGui.QColor(180,180,180)))
	p.setFont(QtGui.QFont("Sans", 14))
	p.drawText(8, canvas.height()-20, "Left-click starts new 'snake' life. Bump its 'head' into a cyan tail.")
	p.end()

canvas.paintEvent = canvas_paintEvent

def buildHeaderText():
	parts = []
	if analysisResult is None:
		parts.append("no contour analyzed")
	else:
		n = analysisResult
		nibble, angleBin, rotationBit = Nibble_from_Analysis(n)
		parts.append(f"angle={n['angleDeg_head_to_half']:.1f}°")
		parts.append(f"{n['orientation']}")
		parts.append(f"roundInv={n['roundnessInv']:.3f}")
		parts.append(f"nibble=0x{nibble:01X}")
		if BiteBuffer is not None:
			parts.append(f"first=0x{BiteBuffer:01X}")
		if lastGestureInfo is not None and lastGestureInfo.get('combinedByte') is not None:
			cb = lastGestureInfo['combinedByte']
			parts.append(f"byte=0x{cb:02X}")
	return "  ".join(parts)

# ---------- snake trimming and self-bite ----------
def trimSnakeToMaxLength():
	global SnakePoints
	if len(SnakePoints) < 2:
		return
	cum = CumulativeDistances_from_Head(SnakePoints)
	lastIdx = len(SnakePoints)-1
	for idx, d in enumerate(cum):
		if d > maxSnakeAccumulativeLength:
			lastIdx = max(0, idx-1)
			break
	if lastIdx < len(SnakePoints)-1:
		SnakePoints = SnakePoints[:lastIdx+1]

def checkSelfBite():
	global SnakePoints, analysisResult, BiteBuffer, lastGestureInfo
	if len(SnakePoints) < 6:
		return
	cum = CumulativeDistances_from_Head(SnakePoints)
	cyanStartIdx = None
	for i, d in enumerate(cum):
		if d >= minSnakeAccumulativeLength:
			cyanStartIdx = i
			break
	if cyanStartIdx is None:
		return
	head = SnakePoints[0]
	for idx in range(cyanStartIdx, len(SnakePoints)):
		pt = SnakePoints[idx]
		if Dist_between_Points(head, pt) <= collisionThreshold:
			# collision -> contour head..idx inclusive
			contourSegment = SnakePoints[0:idx+1]
			if len(contourSegment) >= 3:
				try:
					res = AnalyzeClosedContour_from_SnakeSegment(contourSegment)
				except Exception as e:
					analysisResult = None
					lastGestureInfo = None
					SnakePoints = []
					return
				analysisResult = res
				nibble, angleBin, rotationBit = Nibble_from_Analysis(res)
				# logic:
				# if inverted roundness < lineThreshold -> emit nibble directly as codepoint
				# else: two-bite composition
				if res['roundnessInv'] <= lineThreshold:
					# single-bite direct nibble emission as codepoint
					byteVal = nibble & 0xFF
					try:
						ch = chr(byteVal)
					except Exception:
						ch = '\uFFFD'
					# backspace handling if nibble==0x08
					if byteVal == 0x08:
						txt = textEdit.toPlainText()
						if len(txt) > 0:
							textEdit.setPlainText(txt[:-1])
							cursor = textEdit.textCursor()
							cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
							textEdit.setTextCursor(cursor)
					else:
						textEdit.insertPlainText(ch)
					lastGestureInfo = {'single': True, 'nibble': nibble, 'emittedByte': byteVal, 'action': 'EMIT_NIBBLE_DIRECT'}
					BiteBuffer = None
				else:
					# two-bite composition
					if BiteBuffer is None:
						BiteBuffer = (nibble & 0xF)
						lastGestureInfo = {'single': False, 'nibble': nibble, 'waiting': True}
					else:
						low		= BiteBuffer		& 0xF
						high	= nibble			& 0xF
						byteVal = ((high << 4) | low)	& 0xFF
						try:
							ch = chr(byteVal)
						except Exception:
							ch = '\uFFFD'
						if byteVal == 0x08:
							txt = textEdit.toPlainText()
							if len(txt) > 0:
								textEdit.setPlainText(txt[:-1])
								cursor = textEdit.textCursor()
								cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
								textEdit.setTextCursor(cursor)
						else:
							textEdit.insertPlainText(ch)
						lastGestureInfo = {'single': False, 'nibble': nibble, 'combinedByte': byteVal}
						BiteBuffer = None
			else:
				analysisResult = None
				lastGestureInfo = None
			# reset snake after detection
			SnakePoints = []
			return

# ---------- mouse events ----------
def canvas_mousePressEvent(ev):
	global SnakePoints, recording, BiteBuffer
	if ev.button() == QtCore.Qt.MouseButton.LeftButton:
		# start new snake life
		SnakePoints = [(ev.position().x(), ev.position().y())]
		recording = True
		# do not clear BiteBuffer (user wants multi-bite sequences across lives)
		# clear last gesture overlay
		# analysisResult = None  # keep last analysis visible
		canvas.update()
	elif ev.button() == QtCore.Qt.MouseButton.RightButton:
		# cancel current life
		SnakePoints = []
		recording = False
		canvas.update()

def canvas_mouseMoveEvent(ev):
	global SnakePoints
	if not recording:
		return
	pos = (ev.position().x(), ev.position().y())
	if len(SnakePoints) == 0 or Dist_between_Points(SnakePoints[0], pos) > 0.6:
		SnakePoints.insert(0, pos)
	trimSnakeToMaxLength()
	checkSelfBite()
	canvas.update()

canvas.mousePressEvent = canvas_mousePressEvent
canvas.mouseMoveEvent = canvas_mouseMoveEvent

# repaint timer
repaintTimer = QtCore.QTimer()
def onRepaintTimer():
	canvas.update()
repaintTimer.timeout.connect(onRepaintTimer)
repaintTimer.start(30)

# ---------- run ----------
OAppO.show()
OApp.exec()
