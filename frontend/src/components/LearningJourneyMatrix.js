import React from 'react';
import Link from '@docusaurus/Link';

const matrixData = [
  {
    module: 1,
    title: "Foundations",
    description: "Master the physical AI architecture, ROS 2 core concepts, and hardware interfaces.",
    link: "/docs/module1/architecture-concepts",
  },
  {
    module: 2,
    title: "Simulation",
    description: "Build robust worlds in Gazebo and model physics for accurate sensor simulation.",
    link: "/docs/module2/gazebo-setup",
  },
  {
    module: 3,
    title: "Perception",
    description: "Implement vSLAM, Navigation 2 stacks, and spatial awareness for humanoids.",
    link: "/docs/module3/isaac-sim-intro",
  },
  {
    module: 4,
    title: "Cognition",
    description: "Integrate LLMs, VLA models, and cognitive planning for true autonomy.",
    link: "/docs/module4/conversational-robotics",
  }
];

export default function LearningJourneyMatrix() {
  return (
    <section className="matrix-section padding-vert--xl">
      <div className="container">
        <div className="text--center margin-bottom--lg">
          <h2 className="section-title">The Four Modules</h2>
          <div className="section-divider" />
          <p className="section-description">
            A structured breakdown of the comprehensive curriculum, from foundational architecture to cognitive autonomy.
          </p>
        </div>

        <div className="row">
          {matrixData.map((item, idx) => (
            <div key={idx} className="col col--3 margin-bottom--lg">
              <div className="matrix-card">
                <div className="matrix-card-header">Module {item.module}</div>
                <h3 className="matrix-card-title">{item.title}</h3>
                <p className="matrix-card-desc">{item.description}</p>
                <Link to={item.link} className="matrix-card-link">
                  Start Module &rarr;
                </Link>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
