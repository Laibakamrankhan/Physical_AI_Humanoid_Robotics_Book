import React from 'react';
import { SparklesIcon } from '@heroicons/react/24/outline';
import Link from '@docusaurus/Link';

export default function ModuleCard({ title, desc, icon }) {
return (
<div className="module-card">
<div className="module-card-icon">{icon || <SparklesIcon />}</div>
<h3 className="module-card-title">{title}</h3>
<p className="module-card-desc">{desc}</p>
<Link
    className="module-card-link"
    to="/docs/physical-ai-robotics/Modules/module1-contract"
  >
    Explore →
  </Link>
</div>
);
}