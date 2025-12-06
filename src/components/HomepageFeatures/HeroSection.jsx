import React from 'react';
import Link from '@docusaurus/Link';

export default function HeroSection() {
  return (
    <section className="hero-section">
      <div className="hero-overlay">
        <div className="hero-content container">
          
          {/* Text Section */}
          <div className="hero-text">
            <h1>Physical AI & Humanoid Robotics</h1>
            <p className="tagline">
              A Practical Guide to Physical AI and Humanoid Systems
            </p>

            <div className="cta-row">
             <Link to="/CH01-ros2-nervous-system" className="btn btn-primary">
                   Start Learning
            </Link>
              <a href="/signup" className="btn btn-ghost">Sign Up</a>
            </div>
          </div>

        </div>
      </div>
    </section>
  );
}
