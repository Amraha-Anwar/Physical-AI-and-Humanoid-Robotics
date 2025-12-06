import React from 'react';
import Link from '@docusaurus/Link';

export default function ModuleCard({ title, description, link, icon }) {
  return (
    <Link
      to={link}
      className="module-card"
    >
      {/* Glow effect on hover */}
      <div className="module-card-glow" />
        
      <div className="module-card-content">
        {icon && (
          <div className="module-card-icon">
            {icon}
          </div>
        )}
          
        <h3 className="module-card-title">
          {title}
        </h3>
          
        <p className="module-card-description">
          {description}
        </p>
      </div>
    </Link>
  );
}