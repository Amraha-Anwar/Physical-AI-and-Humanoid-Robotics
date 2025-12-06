import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '../components/HeroSection';
import ModuleCard from '../components/ModuleCard';
import LearningJourneyMatrix from '../components/LearningJourneyMatrix';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Master the full stack of Physical AI and Humanoid Robotics.">
      
      {/* Hero Section */}
      <HeroSection 
        title="Physical AI & Humanoid Robotics"
        subtitle="From Simulation to Reality: A complete guide to building embodied intelligence."
        ctaText="Start Learning"
        ctaLink="/docs/intro/foundations"
        imageSrc="/img/hero.png"
      />

      <main className="container padding-vert--xl">
        
        {/* The Four Modules (Breakdown & Scope) */}
        <LearningJourneyMatrix />
        
        {/* Core Competencies (Learning Outcomes) - Asymmetrical Grid */}
        <section className="margin-bottom--xl">
          <div className="text--center margin-bottom--lg">
            <h2 className="section-title">
              Core Competencies
            </h2>
            <div className="section-divider" />
            <p className="section-description">
              Achieve mastery in the critical skills required to build, simulate, and deploy autonomous humanoid robots.
            </p>
          </div>

          <div className="competency-grid">
            <ModuleCard 
              title="Physical Intelligence"
              description="Design and control embodied agents that interact dynamically with the real world."
              link="/docs/module1/architecture-concepts"
              icon="⚡"
            />
            <ModuleCard 
              title="Sim-to-Real Transfer"
              description="Bridge the gap between high-fidelity simulation and physical hardware deployment."
              link="/docs/module2/gazebo-setup"
              icon="🔄"
            />
            <ModuleCard 
              title="Cognitive Reasoning"
              description="Implement multimodal LLMs and VLAs for high-level planning and decision making."
              link="/docs/module4/conversational-robotics"
              icon="🧠"
            />
            <ModuleCard 
              title="Spatial Perception"
              description="Master vSLAM and semantic understanding for robust navigation in unstructured environments."
              link="/docs/module3/isaac-sim-intro"
              icon="👁️"
            />
             <ModuleCard 
              title="Full-Stack Deployment"
              description="Architect production-grade ROS 2 systems with containerization and orchestration."
              link="/docs/intro/foundations"
              icon="🚀"
            />
          </div>
        </section>

      </main>
    </Layout>
  );
}
