import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '../components/HeroSection';
import ModuleCard from '../components/ModuleCard';

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
        
        {/* Master the Stack Section */}
        <section className="margin-bottom--xl">
          <div className="text--center margin-bottom--lg">
            <h2 className="section-title">
              Master the Stack
            </h2>
            <div className="section-divider" />
            <p className="section-description">
              A comprehensive curriculum designed to take you from core concepts to deploying autonomous agents on humanoid hardware.
            </p>
          </div>

          <div className="grid-layout">
            <ModuleCard 
              title="Module 1: Foundations"
              description="ROS 2, URDF modeling, and the architecture of robotic systems."
              link="/docs/module1/architecture-concepts"
              icon="🏗️"
            />
            <ModuleCard 
              title="Module 2: Simulation"
              description="Master physics, sensors, and environments in Gazebo & Isaac Sim."
              link="/docs/module2/gazebo-setup"
              icon="🌐"
            />
            <ModuleCard 
              title="Module 3: Perception"
              description="VSLAM, Navigation 2, and spatial awareness for humanoids."
              link="/docs/module3/isaac-sim-intro"
              icon="👁️"
            />
            <ModuleCard 
              title="Module 4: Cognition"
              description="Integrate LLMs, VLAs, and planning for true autonomy."
              link="/docs/module4/conversational-robotics"
              icon="🧠"
            />
          </div>
        </section>

      </main>
    </Layout>
  );
}