import React from 'react';
import Link from '@docusaurus/Link';

export default function HeroSection({ title, subtitle, ctaText, ctaLink, imageSrc }) {
  return (
    <section className="hero-section">
      {/* Background gradients */}
      <div className="hero-background-gradient-top" />
      <div className="hero-background-gradient-bottom" />

      <div className="container hero-content-container">
        {/* Text Content */}
        <div className="hero-text-content">
          <h1 className="hero-title">
            {title.split(' ').map((word, i) => (
              <span key={i} className={i === 0 ? "text-white" : "hero-title-highlight"}>
                {word}{' '}
              </span>
            ))}
          </h1>
          
          <p className="hero-subtitle">
            {subtitle}
          </p>

          <div>
            <Link
              to={ctaLink}
              className="hero-cta-button"
            >
              {ctaText}
            </Link>
          </div>
        </div>

        {/* Hero Image */}
        <div className="hero-image-container">
          <div className="hero-image-wrapper">
             {/* Glow behind image */}
            <div className="hero-image-glow" />
            
            <img 
              src={imageSrc} 
              alt="Physical AI Robot" 
              className="hero-image"
            />
          </div>
        </div>
      </div>
    </section>
  );
}