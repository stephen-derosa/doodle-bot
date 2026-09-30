"use client";

import { useEffect, useRef } from "react";

const PX_PER_MM = 5;

const WIDTH_MM = 100;
const HEIGHT_MM = 150;

const WIDTH_PX = WIDTH_MM * PX_PER_MM;
const HEIGHT_PX = HEIGHT_MM * PX_PER_MM;


export default function Whiteboard() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const isDrawing = useRef(false);

  useEffect(() => {
    const ctx = canvasRef.current?.getContext("2d");
    if (!ctx) return;
    ctx.strokeStyle = "black";
    ctx.lineWidth = 3;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
  }, []);

  function getPoint(e: React.PointerEvent<HTMLCanvasElement>) {
    const rect = e.currentTarget.getBoundingClientRect();
    return {
      x: ((e.clientX - rect.left) * WIDTH_PX) / rect.width,
      y: (((e.clientY - rect.top) * HEIGHT_PX) / rect.height),
    };
  }

  function handlePointerDown(e: React.PointerEvent<HTMLCanvasElement>) {
    const ctx = e.currentTarget.getContext("2d");
    if (!ctx) return;
    e.currentTarget.setPointerCapture(e.pointerId);
    isDrawing.current = true;
    const { x, y } = getPoint(e);
    ctx.beginPath();
    ctx.moveTo(x, y);
  }

  function handlePointerMove(e: React.PointerEvent<HTMLCanvasElement>) {
    if (!isDrawing.current) return;
    const ctx = e.currentTarget.getContext("2d");
    if (!ctx) {
      return;
    }

    const { x, y } = getPoint(e);

    const xInMm = x / PX_PER_MM;
    const yInMm = (HEIGHT_PX - y) / PX_PER_MM;

    const xInMmBounded = Math.min(Math.max(0, xInMm), WIDTH_PX);
    const yInMmBounded = Math.min(Math.max(0, yInMm), HEIGHT_PX);

    console.log(`x ${Math.round(xInMmBounded)}, y: ${Math.round(yInMmBounded)}`);

    ctx.lineTo(x, y);
    ctx.stroke();
  }

  function handlePointerUp() {
    isDrawing.current = false;
  }

  return (
    <canvas
      ref={canvasRef}
      width={WIDTH_PX}
      height={HEIGHT_PX}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerUp}
      className="w-full bg-white border border-zinc-300 shadow-sm touch-none cursor-crosshair"
      style={{ width: WIDTH_PX, height: HEIGHT_PX }}
    />
  );
}
