import React, { useState, useEffect } from 'react';


// Simple placeholder auth/profile logic using local storage
export default function PersonalizationPanel() {
const [user, setUser] = useState(null);


useEffect(() => {
// Try to load dummy profile
try {
const raw = localStorage.getItem('ui_profile');
if (raw) setUser(JSON.parse(raw));
} catch (e) {
// ignore
}
}, []);


const signInDemo = () => {
const demo = { name: 'Alex', background: 'controls' };
localStorage.setItem('ui_profile', JSON.stringify(demo));
setUser(demo);
};


const signOut = () => {
localStorage.removeItem('ui_profile');
setUser(null);
};


const recommended = user ? ["Module 2: Control & Planning", "Module 4: VLA"] : ["Module 1: Foundations", "Module 3: Perception"];


return (
<div className="personalization-panel">
<h3>Recommendations</h3>
{user ? (
<div>
<p className="greeting">Welcome back, <strong>{user.name}</strong>! Ready to explore {recommended[0]}?</p>
<ul className="rec-list">
{recommended.map((r) => (
<li key={r}>{r}</li>
))}
</ul>
<button className="btn btn-ghost" onClick={signOut}>Sign Out</button>
</div>
) : (
<div>
<p className="greeting">Welcome! Create a free profile to get personalized recommendations.</p>
<button className="btn btn-primary" onClick={signInDemo}>Sign in (demo)</button>
</div>
)}
</div>
);
}