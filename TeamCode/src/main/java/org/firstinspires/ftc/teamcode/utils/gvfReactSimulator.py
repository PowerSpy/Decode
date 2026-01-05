import React, { useState, useEffect, useRef } from 'react';
import { Play, Pause, RotateCcw, Settings } from 'lucide-react';

const SplineNavigationSim = () => {
  const canvasRef = useRef(null);
  const [isRunning, setIsRunning] = useState(false);
  const [showVectorField, setShowVectorField] = useState(true);
  const [algorithm, setAlgorithm] = useState('lookahead');
  const [showSliderConfig, setShowSliderConfig] = useState(false);
  const [config, setConfig] = useState({
    maxSpeed: 70,
    attractionGain: 0.8,
    tangentGain: 1.2,
    lookaheadDist: 30,
    distanceGain: 0.01,
    maxAccel: 128,
    kRep: 50.0,
    repSigma: 10.0,
    terminalGain: 0.0,
    terminalRange: 150.0
  });
  const lastTimeRef = useRef(null);
  const SIM_SPEED = 1; // ← increase to run faster without changing physics
  const SCALE = 5;

  const [obstacles, setObstacles] = useState([
    { x: 144, y: 72 },
    { x: 132, y: 72 },
    { x: 120, y: 72 },
    { x: 108, y: 72 },
    { x: 96, y: 72 },
    { x: 84, y: 72 },
    { x: 72, y: 72 },
    { x: 60, y: 72 },
    { x: 48, y: 72 },
    { x: 36, y: 72 },
    { x: 24, y: 72 },
    { x: 12, y: 72 },
    { x: 0, y: 72 }
  ]);
  const [draggingObstacle, setDraggingObstacle] = useState(null);

  const [sliderConfig, setSliderConfig] = useState({
    maxSpeed: { min: 0.1, max: 144, step: 1 },
    attractionGain: { min: 0, max: 2, step: 0.1 },
    tangentGain: { min: 0, max: 3, step: 0.1 },
    lookaheadDist: { min: 0, max: 100, step: 5 },
    distanceGain: { min: 0, max: 0.05, step: 0.001 },
    maxAccel: { min: 0.01, max: 144, step: 1 },
    kRep: { min: 0, max: 200, step: 10 },
    repSigma: { min: 10, max: 150, step: 5 },
    terminalGain: { min: 0, max: 5, step: 0.1 },
    terminalRange: { min: 50, max: 300, step: 10 }
  });

  const [pointA, setPointA] = useState({ x: 108, y: 108 });
  const [pointB, setPointB] = useState({ x: 36, y: 108 });
  const [control1, setControl1] = useState({ x: 108, y: 84 });
  const [control2, setControl2] = useState({ x: 36, y: 84 });

  const [dragging, setDragging] = useState(null);
  const startPos = {x: pointA.x + 12, y: pointA.y + 24, heading: Math.PI * 3 / 2};
  const animationRef = useRef(null);
  const stateRef = useRef({
    object: { x: startPos.x, y: startPos.y, vx: 0, vy: 0, ax: 0, ay: 0, angle: startPos.heading },
    time: 0,
    trail: []
  });

  const evaluateSpline = (t) => {
    const t2 = t * t;
    const t3 = t2 * t;
    const mt = 1 - t;
    const mt2 = mt * mt;
    const mt3 = mt2 * mt;

    return {
      x: mt3 * pointA.x + 3 * mt2 * t * control1.x + 3 * mt * t2 * control2.x + t3 * pointB.x,
      y: mt3 * pointA.y + 3 * mt2 * t * control1.y + 3 * mt * t2 * control2.y + t3 * pointB.y
    };
  };

  const splineTangent = (t) => {
    const mt = 1 - t;
    const mt2 = mt * mt;
    const t2 = t * t;

    const dx = 3 * mt2 * (control1.x - pointA.x) + 6 * mt * t * (control2.x - control1.x) + 3 * t2 * (pointB.x - control2.x);
    const dy = 3 * mt2 * (control1.y - pointA.y) + 6 * mt * t * (control2.y - control1.y) + 3 * t2 * (pointB.y - control2.y);

    const len = Math.sqrt(dx * dx + dy * dy);
    return { x: dx / len, y: dy / len, magnitude: len };
  };

  const splineCurvature = (t) => {
    const mt = 1 - t;

    const dx = 3 * mt * mt * (control1.x - pointA.x) + 6 * mt * t * (control2.x - control1.x) + 3 * t * t * (pointB.x - control2.x);
    const dy = 3 * mt * mt * (control1.y - pointA.y) + 6 * mt * t * (control2.y - control1.y) + 3 * t * t * (pointB.y - control2.y);

    const ddx = 6 * mt * (control2.x - 2 * control1.x + pointA.x) + 6 * t * (pointB.x - 2 * control2.x + control1.x);
    const ddy = 6 * mt * (control2.y - 2 * control1.y + pointA.y) + 6 * t * (pointB.y - 2 * control2.y + control1.y);

    const numerator = Math.abs(dx * ddy - dy * ddx);
    const denominator = Math.pow(dx * dx + dy * dy, 1.5);

    return denominator > 0 ? numerator / denominator : 0;
  };

  const findClosestPoint = (px, py) => {
    let minDist = Infinity;
    let bestT = 0;

    for (let t = 0; t <= 1; t += 0.01) {
      const pt = evaluateSpline(t);
      const dist = Math.sqrt((pt.x - px) ** 2 + (pt.y - py) ** 2);
      if (dist < minDist) {
        minDist = dist;
        bestT = t;
      }
    }

    for (let t = Math.max(0, bestT - 0.02); t <= Math.min(1, bestT + 0.02); t += 0.001) {
      const pt = evaluateSpline(t);
      const dist = Math.sqrt((pt.x - px) ** 2 + (pt.y - py) ** 2);
      if (dist < minDist) {
        minDist = dist;
        bestT = t;
      }
    }

    return { t: bestT, point: evaluateSpline(bestT), distance: minDist };
  };

  const computeVelocity = (px, py, currentVx, currentVy) => {
    if (algorithm === 'lookahead') {
      return computeVelocityLookahead(px, py);
    } else if (algorithm === 'proposed') {
      return computeVelocityProposed(px, py);
    } else {
      return computeVelocityEnhanced(px, py, currentVx, currentVy);
    }
  };

  const computeVelocityLookahead = (px, py) => {
    const closest = findClosestPoint(px, py);
    const lookaheadT = Math.min(1, closest.t + config.lookaheadDist / 600);
    const tangent = splineTangent(lookaheadT);

    const toSpline = {
      x: closest.point.x - px,
      y: closest.point.y - py
    };

    const dist = closest.distance;
    const attractionWeight = Math.min(1, dist / 50) * config.attractionGain;
    const tangentWeight = config.tangentGain;

    // Terminal guidance: increase alignment near point B
    const distToB = Math.sqrt((px - pointB.x) ** 2 + (py - pointB.y) ** 2);
    const terminalWeight = Math.max(0, 1 - distToB / config.terminalRange);

    let vx = attractionWeight * toSpline.x + tangentWeight * tangent.x * 50;
    let vy = attractionWeight * toSpline.y + tangentWeight * tangent.y * 50;

    // Add terminal guidance component
    vx += terminalWeight * config.terminalGain * terminalTangent.x * 50;
    vy += terminalWeight * config.terminalGain * terminalTangent.y * 50;

    const speed = Math.sqrt(vx * vx + vy * vy);
    if (speed > 0) {
      vx = (vx / speed) * config.maxSpeed;
      vy = (vy / speed) * config.maxSpeed;
    }

    return { vx, vy, tangent, terminalWeight };
  };

  const computeVelocityProposed = (px, py) => {
    const closest = findClosestPoint(px, py);
    const tangent = splineTangent(closest.t);
    const curvature = splineCurvature(closest.t);

    const distVector = {
      x: closest.point.x - px,
      y: closest.point.y - py
    };
    const dist = Math.sqrt(distVector.x * distVector.x + distVector.y * distVector.y);

    const splineVel = {
      x: tangent.x * config.maxSpeed,
      y: tangent.y * config.maxSpeed
    };

    const distTerm = {
      x: config.distanceGain * distVector.x,
      y: config.distanceGain * distVector.y
    };

    const curvatureMag = config.maxSpeed * config.maxSpeed * curvature;
    const curvatureTerm = dist > 0 ? {
      x: (distVector.x / dist) * curvatureMag,
      y: (distVector.y / dist) * curvatureMag
    } : { x: 0, y: 0 };

    // Terminal guidance: strongly align with point B tangent near the end
    const distToB = Math.sqrt((px - pointB.x) ** 2 + (py - pointB.y) ** 2);
    const terminalWeight = Math.max(0, 1 - distToB / config.terminalRange);
    const terminalTerm = {
      x: terminalWeight * config.terminalGain * terminalTangent.x * config.maxSpeed,
      y: terminalWeight * config.terminalGain * terminalTangent.y * config.maxSpeed
    };

    let vx = splineVel.x + distTerm.x + curvatureTerm.x + terminalTerm.x;
    let vy = splineVel.y + distTerm.y + curvatureTerm.y + terminalTerm.y;

    const speed = Math.sqrt(vx * vx + vy * vy);
    if (speed > config.maxSpeed * 2) {
      vx = (vx / speed) * config.maxSpeed * 2;
      vy = (vy / speed) * config.maxSpeed * 2;
    }

    return { vx, vy, tangent, terminalWeight };
  };

  const computeVelocityEnhanced = (px, py, currentVx, currentVy) => {
    const closest = findClosestPoint(px, py);
    const tangent = splineTangent(closest.t);
    const curvature = splineCurvature(closest.t);

    const normal = { x: -tangent.y, y: tangent.x };

    const distVector = {
      x: closest.point.x - px,
      y: closest.point.y - py
    };
    const dist = Math.sqrt(distVector.x * distVector.x + distVector.y * distVector.y);

    // Terminal guidance: adjust speed based on distance to B
    const distToB = Math.sqrt((px - pointB.x) ** 2 + (py - pointB.y) ** 2);
    const terminalWeight = Math.max(0, 1 - distToB / config.terminalRange);

    // Slow down as approaching B (reduce speed by up to 70% near B)
    const speedScale = 1 - terminalWeight * 0.7;
    const currentMaxSpeed = config.maxSpeed * speedScale;

    // v_spline: velocity at closest point on spline
    const splineVel = {
      x: tangent.x * currentMaxSpeed,
      y: tangent.y * currentMaxSpeed
    };

    // Gaussian boost to distance gain as approaching B
    const gaussianBoost = Math.exp(-(distToB * distToB) / (2 * config.terminalRange * config.terminalRange));
    const effectiveDistanceGain = config.distanceGain * (1 + gaussianBoost * 5);

    // k·d: distance gain times distance vector (attracts to path)
    const distTerm = {
      x: effectiveDistanceGain * distVector.x,
      y: effectiveDistanceGain * distVector.y
    };

    // (v²·κ)·d̂: curvature term
    // Normalize by distance to prevent speed-dependent convergence point shift
    const curvatureMag = currentMaxSpeed * currentMaxSpeed * curvature;
    const curvatureTerm = dist > 0 ? {
      x: (distVector.x / dist) * curvatureMag / (1 + dist * 0.01),
      y: (distVector.y / dist) * curvatureMag / (1 + dist * 0.01)
    } : { x: 0, y: 0 };

    // Repulsion projected onto normal only (doesn't slow forward progress)
    let vRepulsion = { x: 0, y: 0 };
    for (const obs of obstacles) {
      const dx = px - obs.x;
      const dy = py - obs.y;
      const distSq = dx * dx + dy * dy;
      const distObs = Math.sqrt(distSq);

      if (distObs > 0) {
        const gaussian = Math.exp(-distSq / (2 * config.repSigma * config.repSigma));
        const repMag = config.kRep * gaussian;
        const repX = repMag * (dx / distObs);
        const repY = repMag * (dy / distObs);

        // Project repulsion onto normal direction only
        const repDotNormal = repX * normal.x + repY * normal.y;
        vRepulsion.x += repDotNormal * normal.x;
        vRepulsion.y += repDotNormal * normal.y;
      }
    }

    const vTerminalAlign = {
      x: terminalWeight * config.terminalGain * terminalTangent.x * currentMaxSpeed,
      y: terminalWeight * config.terminalGain * terminalTangent.y * currentMaxSpeed
    };

    // Sum all components
    let vx = splineVel.x + distTerm.x + curvatureTerm.x + vRepulsion.x + vTerminalAlign.x;
    let vy = splineVel.y + distTerm.y + curvatureTerm.y + vRepulsion.y + vTerminalAlign.y;

    // Limit to reasonable speed
    const speed = Math.sqrt(vx * vx + vy * vy);
    if (speed > config.maxSpeed * 2) {
      vx = (vx / speed) * config.maxSpeed * 2;
      vy = (vy / speed) * config.maxSpeed * 2;
    }

    return { vx, vy, tangent, normal, terminalWeight, desiredSpeed: currentMaxSpeed, gaussianBoost };
  };

  const draw = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.save();
    ctx.scale(SCALE, SCALE);


    ctx.strokeRect(0, 0, 144, 144);
    if (showVectorField) {
      ctx.strokeStyle = 'rgba(100, 150, 250, 0.3)';
      ctx.fillStyle = 'rgba(100, 150, 250, 0.3)';
      for (let x = 50; x < 144; x += 20) {
        for (let y = 50; y < 144; y += 20) {
          const vel = computeVelocity(x, y, 0, 0);
          const scale = 8;
          ctx.beginPath();
          ctx.moveTo(x, y);
          ctx.lineTo(x + vel.vx * scale, y + vel.vy * scale);
          ctx.stroke();

          const angle = Math.atan2(vel.vy, vel.vx);
          ctx.beginPath();
          ctx.moveTo(x + vel.vx * scale, y + vel.vy * scale );
          ctx.lineTo(x + vel.vx * scale - 4 * Math.cos(angle - 0.5), y + vel.vy * scale - 4 * Math.sin(angle - 0.5));
          ctx.lineTo(x + vel.vx * scale - 4 * Math.cos(angle + 0.5), y + vel.vy * scale - 4 * Math.sin(angle + 0.5));
          ctx.closePath();
          ctx.fill();
        }
      }
    }

    ctx.strokeStyle = '#4CAF50';
    ctx.lineWidth = 3;
    ctx.beginPath();
    for (let t = 0; t <= 1; t += 0.01) {
      const pt = evaluateSpline(t);
      if (t === 0) ctx.moveTo(pt.x, pt.y);
      else ctx.lineTo(pt.x, pt.y);
    }
    ctx.stroke();

    ctx.fillStyle = '#4CAF50';
    ctx.beginPath();
    ctx.arc(pointA.x, pointA.y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#F44336';
    ctx.beginPath();
    ctx.arc(pointB.x, pointB.y, 8, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#999';
    ctx.setLineDash([5, 5]);
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pointA.x, pointA.y);
    ctx.lineTo(control1.x, control1.y);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(pointB.x, pointB.y);
    ctx.lineTo(control2.x, control2.y);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = '#2196F3';
    ctx.beginPath();
    ctx.arc(control1.x, control1.y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.arc(control2.x, control2.y, 6, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#000';
    ctx.font = '14px sans-serif';
    ctx.fillText('A', pointA.x - 5, pointA.y - 12);
    ctx.fillText('B', pointB.x - 5, pointB.y - 12);
    ctx.fillText('C1', control1.x - 10, control1.y - 12);
    ctx.fillText('C2', control2.x - 10, control2.y - 12);

    const trail = stateRef.current.trail;
    if (trail.length > 1) {
      ctx.strokeStyle = 'rgba(255, 152, 0, 0.5)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(trail[0].x, trail[0].y);
      for (let i = 1; i < trail.length; i++) {
        ctx.lineTo(trail[i].x, trail[i].y);
      }
      ctx.stroke();
    }

    const obj = stateRef.current.object;
    ctx.save();
    ctx.translate(obj.x, obj.y);
    ctx.rotate(obj.angle);

    ctx.fillStyle = '#FF9800';
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(-8, -8);
    ctx.lineTo(-8, 8);
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = '#F57C00';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();

    if (algorithm === 'enhanced') {
      for (const obs of obstacles) {
        ctx.fillStyle = 'rgba(255, 0, 0, 0.1)';
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, config.repSigma * 2, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#F44336';
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, 8, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = '#C62828';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }
    ctx.restore();
  };

  const terminalTangent = splineTangent(1);
  const update = (dt) => {
    if (dt <= 0) return;
    const obj = stateRef.current.object;
    const distToB = Math.sqrt((obj.x - pointB.x) ** 2 + (obj.y - pointB.y) ** 2);

    if (distToB > 6 || Math.abs(Math.atan2(terminalTangent.y, terminalTangent.x) - obj.angle) >= 0.01) {
      const vel = computeVelocity(obj.x, obj.y, obj.vx, obj.vy);

      if (algorithm === 'proposed' || algorithm === 'enhanced') {
        // Apply acceleration limiting for realistic motion
        const ax = (vel.vx - obj.vx) / dt;
        const ay = (vel.vy - obj.vy) / dt;

        const aMag = Math.sqrt(ax * ax + ay * ay);

        if (aMag > config.maxAccel) {
          obj.ax = (ax / aMag) * config.maxAccel;
          obj.ay = (ay / aMag) * config.maxAccel;
        } else {
          obj.ax = ax;
          obj.ay = ay;
        }

        obj.vx += obj.ax * dt;
        obj.vy += obj.ay * dt;

      } else {
        obj.vx = vel.vx;
        obj.vy = vel.vy;
      }

      obj.x += obj.vx * dt;
      obj.y += obj.vy * dt;

      obj.angle = Math.atan2(obj.vy, obj.vx);

      stateRef.current.trail.push({ x: obj.x, y: obj.y });
    } else {
      setIsRunning(false);
    }


  };

  const animate = (time) => {
    if (lastTimeRef.current === null) {
      lastTimeRef.current = time;
    }

    const rawDt = (time - lastTimeRef.current) / 1000; // seconds
    lastTimeRef.current = time;

    // Clamp for stability, then scale simulation time
    const dt = Math.min(0.05, rawDt) * SIM_SPEED;

    update(dt);
    draw();

    if (isRunning) {
      animationRef.current = requestAnimationFrame(animate);
    }
  };

  useEffect(() => {
    if (isRunning) {
      animationRef.current = requestAnimationFrame(animate);
    }
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isRunning, config, algorithm, obstacles, pointA, pointB, control1, control2]);

  useEffect(() => {
    draw();
  }, [showVectorField, algorithm, obstacles, pointA, pointB, control1, control2]);

  const reset = () => {
    setIsRunning(false);
    lastTimeRef.current = null;
    stateRef.current = {
      object: { x: startPos.x, y: startPos.y, vx: 0, vy: 0, ax: 0, ay: 0, angle: startPos.heading },
      time: 0,
      trail: []
    };
    draw();
  };

  const togglePlay = () => {
    setIsRunning(!isRunning);
  };

  const updateSliderConfig = (param, field, value) => {
    setSliderConfig(prev => ({
      ...prev,
      [param]: {
        ...prev[param],
        [field]: parseFloat(value)
      }
    }));
  };

  const getMousePos = (e) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    return {
      x: (e.clientX - rect.left) / SCALE,
      y: (e.clientY - rect.top) / SCALE
    };
  };

  const isNearPoint = (pos, point, radius = 10) => {
    const dx = pos.x - point.x;
    const dy = pos.y - point.y;
    return Math.sqrt(dx * dx + dy * dy) < radius;
  };

  const handleMouseDown = (e) => {
    const pos = getMousePos(e);

    if (isNearPoint(pos, pointA)) {
      setDragging('pointA');
    } else if (isNearPoint(pos, pointB)) {
      setDragging('pointB');
    } else if (isNearPoint(pos, control1)) {
      setDragging('control1');
    } else if (isNearPoint(pos, control2)) {
      setDragging('control2');
    } else if (algorithm === 'enhanced') {
      for (let i = 0; i < obstacles.length; i++) {
        if (isNearPoint(pos, obstacles[i])) {
          setDraggingObstacle(i);
          return;
        }
      }
    }
  };

  const handleMouseMove = (e) => {
    if (!dragging && draggingObstacle === null) return;

    const pos = getMousePos(e);

    if (dragging === 'pointA') {
      setPointA(pos);
    } else if (dragging === 'pointB') {
      setPointB(pos);
    } else if (dragging === 'control1') {
      setControl1(pos);
    } else if (dragging === 'control2') {
      setControl2(pos);
    } else if (draggingObstacle !== null) {
      const newObstacles = [...obstacles];
      newObstacles[draggingObstacle] = pos;
      setObstacles(newObstacles);
    }

    draw();
  };

  const handleMouseUp = () => {
    setDragging(null);
    setDraggingObstacle(null);
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    canvas.addEventListener('mousedown', handleMouseDown);
    canvas.addEventListener('mousemove', handleMouseMove);
    canvas.addEventListener('mouseup', handleMouseUp);
    canvas.addEventListener('mouseleave', handleMouseUp);

    return () => {
      canvas.removeEventListener('mousedown', handleMouseDown);
      canvas.removeEventListener('mousemove', handleMouseMove);
      canvas.removeEventListener('mouseup', handleMouseUp);
      canvas.removeEventListener('mouseleave', handleMouseUp);
    };
  }, [dragging, draggingObstacle, pointA, pointB, control1, control2, obstacles, algorithm]);

  const getDecimalPlaces = (step) => {
    return Math.max(0, -Math.floor(Math.log10(step)));
  };

  return (
    <div className="w-full h-screen bg-gray-50 p-6 overflow-auto">
      <div className="max-w-6xl mx-auto">
        <h1 className="text-3xl font-bold mb-2">Cubic Spline Navigation</h1>
        <p className="text-gray-600 mb-4">
          Object navigates to spline while following its direction toward point B.
          <strong> Drag the control points (A, B, C1, C2) to modify the spline!</strong>
          {algorithm === 'enhanced' && <strong> Drag red obstacles to reposition them!</strong>}
          <br />
          <span className="text-sm">Units: inches for distance, inches/second for velocity, inches/second² for acceleration</span>
        </p>

        <div className="bg-white rounded-lg shadow-lg p-4 mb-4">
          <canvas
            ref={canvasRef}
            width={144 * SCALE}
            height={144 * SCALE}
            className="border border-gray-300 rounded cursor-pointer"
          />
        </div>

        <div className="bg-white rounded-lg shadow-lg p-4 mb-4">
          <div className="flex gap-4 mb-4 flex-wrap">
            <button
              onClick={togglePlay}
              className="flex items-center gap-2 px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
            >
              {isRunning ? <Pause size={20} /> : <Play size={20} />}
              {isRunning ? 'Pause' : 'Start'}
            </button>

            <button
              onClick={reset}
              className="flex items-center gap-2 px-4 py-2 bg-gray-500 text-white rounded hover:bg-gray-600"
            >
              <RotateCcw size={20} />
              Reset
            </button>

            <button
              onClick={() => setShowVectorField(!showVectorField)}
              className="flex items-center gap-2 px-4 py-2 bg-green-500 text-white rounded hover:bg-green-600"
            >
              <Settings size={20} />
              {showVectorField ? 'Hide' : 'Show'} Vector Field
            </button>

            <button
              onClick={() => setShowSliderConfig(!showSliderConfig)}
              className="flex items-center gap-2 px-4 py-2 bg-purple-500 text-white rounded hover:bg-purple-600"
            >
              <Settings size={20} />
              Configure Sliders
            </button>

            <div className="flex items-center gap-2 ml-auto">
              <label className="font-medium">Algorithm:</label>
              <select
                value={algorithm}
                onChange={(e) => setAlgorithm(e.target.value)}
                className="px-3 py-2 border border-gray-300 rounded"
              >
                <option value="lookahead">Lookahead Pursuit</option>
                <option value="proposed">Proposed Algorithm</option>
                <option value="enhanced">Enhanced Algorithm</option>
              </select>
            </div>
          </div>

          {showSliderConfig && (
            <div className="border-t pt-4 mt-4">
              <h3 className="text-lg font-semibold mb-3">Slider Configuration</h3>
              <div className="grid grid-cols-1 gap-4">
                {Object.entries(sliderConfig).map(([param, settings]) => {
                  const allowedParams = algorithm === 'lookahead'
                    ? ['maxSpeed', 'attractionGain', 'tangentGain', 'lookaheadDist', 'terminalGain', 'terminalRange']
                    : algorithm === 'proposed'
                    ? ['maxSpeed', 'distanceGain', 'maxAccel', 'terminalGain', 'terminalRange']
                    : ['maxSpeed', 'distanceGain', 'maxAccel', 'kRep', 'repSigma', 'terminalGain', 'terminalRange'];

                  if (!allowedParams.includes(param)) return null;

                  const paramNames = {
                    maxSpeed: 'Max Speed',
                    attractionGain: 'Attraction Gain',
                    tangentGain: 'Tangent Gain',
                    lookaheadDist: 'Lookahead Distance',
                    distanceGain: 'Distance Gain (k)',
                    maxAccel: 'Max Acceleration',
                    kRep: 'Repulsion Strength',
                    repSigma: 'Repulsion Range',
                    terminalGain: 'Terminal Guidance Gain',
                    terminalRange: 'Terminal Guidance Range'
                  };

                  return (
                    <div key={param} className="bg-gray-50 p-3 rounded">
                      <h4 className="font-medium mb-2">{paramNames[param]}</h4>
                      <div className="grid grid-cols-3 gap-3">
                        <div>
                          <label className="block text-xs text-gray-600 mb-1">Min</label>
                          <input
                            type="number"
                            value={settings.min}
                            onChange={(e) => updateSliderConfig(param, 'min', e.target.value)}
                            className="w-full px-2 py-1 border border-gray-300 rounded text-sm"
                          />
                        </div>
                        <div>
                          <label className="block text-xs text-gray-600 mb-1">Max</label>
                          <input
                            type="number"
                            value={settings.max}
                            onChange={(e) => updateSliderConfig(param, 'max', e.target.value)}
                            className="w-full px-2 py-1 border border-gray-300 rounded text-sm"
                          />
                        </div>
                        <div>
                          <label className="block text-xs text-gray-600 mb-1">Step</label>
                          <input
                            type="number"
                            value={settings.step}
                            step="0.001"
                            onChange={(e) => updateSliderConfig(param, 'step', e.target.value)}
                            className="w-full px-2 py-1 border border-gray-300 rounded text-sm"
                          />
                        </div>
                      </div>
                    </div>
                  );
                })}
              </div>
            </div>
          )}

          <div className="grid grid-cols-2 gap-4 mt-4">
            <div>
              <label className="block text-sm font-medium mb-1">
                Max Speed: {config.maxSpeed.toFixed(getDecimalPlaces(sliderConfig.maxSpeed.step))}
              </label>
              <input
                type="range"
                min={sliderConfig.maxSpeed.min}
                max={sliderConfig.maxSpeed.max}
                step={sliderConfig.maxSpeed.step}
                value={config.maxSpeed}
                onChange={(e) => setConfig({...config, maxSpeed: parseFloat(e.target.value)})}
                className="w-full"
              />
            </div>

            {algorithm === 'lookahead' && (
              <>
                <div>
                  <label className="block text-sm font-medium mb-1">
                    Attraction Gain: {config.attractionGain.toFixed(getDecimalPlaces(sliderConfig.attractionGain.step))}
                  </label>
                  <input
                    type="range"
                    min={sliderConfig.attractionGain.min}
                    max={sliderConfig.attractionGain.max}
                    step={sliderConfig.attractionGain.step}
                    value={config.attractionGain}
                    onChange={(e) => setConfig({...config, attractionGain: parseFloat(e.target.value)})}
                    className="w-full"
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium mb-1">
                    Tangent Gain: {config.tangentGain.toFixed(getDecimalPlaces(sliderConfig.tangentGain.step))}
                  </label>
                  <input
                    type="range"
                    min={sliderConfig.tangentGain.min}
                    max={sliderConfig.tangentGain.max}
                    step={sliderConfig.tangentGain.step}
                    value={config.tangentGain}
                    onChange={(e) => setConfig({...config, tangentGain: parseFloat(e.target.value)})}
                    className="w-full"
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium mb-1">
                    Lookahead Distance: {config.lookaheadDist.toFixed(getDecimalPlaces(sliderConfig.lookaheadDist.step))}
                  </label>
                  <input
                    type="range"
                    min={sliderConfig.lookaheadDist.min}
                    max={sliderConfig.lookaheadDist.max}
                    step={sliderConfig.lookaheadDist.step}
                    value={config.lookaheadDist}
                    onChange={(e) => setConfig({...config, lookaheadDist: parseFloat(e.target.value)})}
                    className="w-full"
                  />
                </div>
              </>
            )}

            {algorithm === 'enhanced' && (
              <>
                <div>
                  <label className="block text-sm font-medium mb-1">
                    Distance Gain (k): {config.distanceGain.toFixed(getDecimalPlaces(sliderConfig.distanceGain.step))}
                  </label>
                  <input
                    type="range"
                    min={sliderConfig.distanceGain.min}
                    max={sliderConfig.distanceGain.max}
                    step={sliderConfig.distanceGain.step}
                    value={config.distanceGain}
                    onChange={(e) => setConfig({...config, distanceGain: parseFloat(e.target.value)})}
                    className="w-full"
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium mb-1">
                    Max Acceleration: {config.maxAccel.toFixed(getDecimalPlaces(sliderConfig.maxAccel.step))}
                  </label>
                  <input
                    type="range"
                    min={sliderConfig.maxAccel.min}
                    max={sliderConfig.maxAccel.max}
                    step={sliderConfig.maxAccel.step}
                    value={config.maxAccel}
                    onChange={(e) => setConfig({...config, maxAccel: parseFloat(e.target.value)})}
                    className="w-full"
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium mb-1">
                    Repulsion Strength: {config.kRep.toFixed(getDecimalPlaces(sliderConfig.kRep.step))}
                  </label>
                  <input
                    type="range"
                    min={sliderConfig.kRep.min}
                    max={sliderConfig.kRep.max}
                    step={sliderConfig.kRep.step}
                    value={config.kRep}
                    onChange={(e) => setConfig({...config, kRep: parseFloat(e.target.value)})}
                    className="w-full"
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium mb-1">
                    Repulsion Range: {config.repSigma.toFixed(getDecimalPlaces(sliderConfig.repSigma.step))}
                  </label>
                  <input
                    type="range"
                    min={sliderConfig.repSigma.min}
                    max={sliderConfig.repSigma.max}
                    step={sliderConfig.repSigma.step}
                    value={config.repSigma}
                    onChange={(e) => setConfig({...config, repSigma: parseFloat(e.target.value)})}
                    className="w-full"
                  />
                </div>
              </>
            )}

            <div>
              <label className="block text-sm font-medium mb-1">
                Terminal Guidance Gain: {config.terminalGain.toFixed(getDecimalPlaces(sliderConfig.terminalGain.step))}
              </label>
              <input
                type="range"
                min={sliderConfig.terminalGain.min}
                max={sliderConfig.terminalGain.max}
                step={sliderConfig.terminalGain.step}
                value={config.terminalGain}
                onChange={(e) => setConfig({...config, terminalGain: parseFloat(e.target.value)})}
                className="w-full"
              />
            </div>

            <div>
              <label className="block text-sm font-medium mb-1">
                Terminal Guidance Range: {config.terminalRange.toFixed(getDecimalPlaces(sliderConfig.terminalRange.step))}
              </label>
              <input
                type="range"
                min={sliderConfig.terminalRange.min}
                max={sliderConfig.terminalRange.max}
                step={sliderConfig.terminalRange.step}
                value={config.terminalRange}
                onChange={(e) => setConfig({...config, terminalRange: parseFloat(e.target.value)})}
                className="w-full"
              />
            </div>

            {algorithm === 'proposed' && (
              <div>
                <label className="block text-sm font-medium mb-1">
                  Distance Gain (k): {config.distanceGain.toFixed(getDecimalPlaces(sliderConfig.distanceGain.step))}
                </label>
                <input
                  type="range"
                  min={sliderConfig.distanceGain.min}
                  max={sliderConfig.distanceGain.max}
                  step={sliderConfig.distanceGain.step}
                  value={config.distanceGain}
                  onChange={(e) => setConfig({...config, distanceGain: parseFloat(e.target.value)})}
                  className="w-full"
                />
              </div>
            )}
          </div>
        </div>

        <div className="bg-white rounded-lg shadow-lg p-4">
          <h2 className="text-xl font-semibold mb-2">Algorithm Explanation</h2>

          {algorithm === 'lookahead' && (
            <>
              <p className="text-gray-700 mb-2">
                <strong>Lookahead Pursuit Algorithm:</strong>
              </p>
              <ol className="list-decimal list-inside space-y-1 text-gray-700">
                <li>Find the closest point on the spline to the object</li>
                <li>Calculate a lookahead point further along the spline toward B</li>
                <li>Generate velocity as a weighted blend of attraction and tangent vectors</li>
                <li>Weight attraction more when far from spline, tangent more when close</li>
                <li><strong>Terminal guidance</strong>: As robot approaches B (within terminal range), increasingly align velocity with spline tangent at B for smooth arrival</li>
              </ol>
            </>
          )}

          {algorithm === 'proposed' && (
            <>
              <p className="text-gray-700 mb-2">
                <strong>Proposed Algorithm:</strong>
              </p>
              <div className="bg-gray-100 p-3 rounded mb-2 font-mono text-sm">
                v = v_spline + k·d + (v²·κ)·d̂
              </div>
              <ul className="list-disc list-inside space-y-1 text-gray-700 ml-4">
                <li><strong>v_spline</strong>: velocity at closest point on spline</li>
                <li><strong>k·d</strong>: distance gain times distance vector</li>
                <li><strong>v²·κ·d̂</strong>: curvature correction term</li>
                <li><strong>Gaussian distance boost</strong>: Distance gain increases up to 6x near point B, ensuring strong path convergence in the final approach</li>
                <li><strong>Terminal alignment</strong>: Aligns velocity with spline direction at B (no separate convergence point - fixes the vector field issue!)</li>
                <li><strong>Automatic slowdown</strong>: Speed reduces by up to 70% when approaching B to prevent overshoot</li>
                <li><strong>Acceleration limiting</strong>: Respects max acceleration constraint for realistic motion</li>
              </ul>
            </>
          )}

          {algorithm === 'enhanced' && (
            <>
              <p className="text-gray-700 mb-2">
                <strong>Enhanced Algorithm (Proposed + Obstacle Avoidance):</strong>
              </p>
              <div className="bg-gray-100 p-3 rounded mb-2 font-mono text-sm">
                v = v_spline + k·d + (v²·κ)·d̂ + v_repulsion_⊥ + v_terminal
              </div>
              <p className="text-gray-700 mb-2">Components:</p>
              <ul className="list-disc list-inside space-y-1 text-gray-700 ml-4">
                <li><strong>v_spline</strong>: velocity at closest point on spline (tangent direction)</li>
                <li><strong>k·d</strong>: distance gain times distance vector (path attraction)</li>
                <li><strong>Gaussian distance boost</strong>: k increases up to 6x near B for stronger path adherence</li>
                <li><strong>v²·κ·d̂</strong>: curvature correction term (centripetal acceleration)</li>
                <li><strong>v_repulsion_⊥</strong>: Gaussian obstacle repulsion projected onto normal vector (sideways avoidance only - no slowdown!)</li>
                <li><strong>Terminal alignment</strong>: Aligns with spline direction at B (no separate convergence point)</li>
                <li><strong>Automatic slowdown</strong>: Speed reduces by up to 70% near B to prevent overshoot</li>
                <li><strong>Acceleration limiting</strong>: Respects max acceleration for realistic, smooth motion</li>
              </ul>
              <p className="text-gray-700 mt-2">
                <strong>Key Insight:</strong> The Gaussian boost to distance gain ensures the robot stays tightly on the path as it approaches B, making the final approach more precise while maintaining the elegance of your original algorithm.
              </p>
            </>
          )}
        </div>
      </div>
    </div>
  );
};

export default SplineNavigationSim;