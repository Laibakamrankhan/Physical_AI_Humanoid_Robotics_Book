import React from 'react';
import ModuleCard from './ModuleCard';
import { AcademicCapIcon, Cog6ToothIcon, CameraIcon, PuzzlePieceIcon } from '@heroicons/react/24/outline';


const modules = [
{ id: 1, title: 'Module 1 — The Robotic Nervous System (ROS 2)', desc: 'Middleware for robot control.', icon: <AcademicCapIcon /> },
{ id: 2, title: 'Module 2 — The Digital Twin (Gazebo & Unity)', desc: 'Physics simulation and environment building.', icon: <Cog6ToothIcon /> },
{ id: 3, title: 'Module 3 — The AI-Robot Brain (NVIDIA Isaac)', desc: 'Advanced perception and training', icon: <CameraIcon /> },
{ id: 4, title: 'Module 4 — Vision-Language-Action (VLA)', desc: ' Convergence of LLMs and robotics.', icon: <PuzzlePieceIcon /> },
];


export default function ModuleOverview() {
return (
<div className="module-overview">
<div className="module-grid">
{modules.map((m) => (
<ModuleCard key={m.id} title={m.title} desc={m.desc} icon={m.icon} />
))}
</div>
</div>
);
}