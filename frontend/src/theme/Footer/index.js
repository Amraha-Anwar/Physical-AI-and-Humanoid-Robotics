import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const footerLinks = [
  {
    title: "Module 1: Foundations",
    to: "/docs/module1/architecture-concepts"
  },
  {
    title: "Module 2: Simulation",
    to: "/docs/module2/gazebo-setup"
  },
  {
    title: "Module 3: Perception",
    to: "/docs/module3/isaac-sim-intro"
  },
  {
    title: "Module 4: Cognition",
    to: "/docs/module4/conversational-robotics"
  }
];

function Footer() {
  const {siteConfig} = useDocusaurusContext();
  const currentYear = new Date().getFullYear();

  return (
    <footer className="custom-footer padding-vert--xl">
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <h2 className="footer-brand-title">{siteConfig.title}</h2>
            <p className="footer-tagline">
              Master the full stack of Physical AI and Humanoid Robotics.
            </p>
          </div>
          <div className="col col--8">
            <div className="row">
              {/* Split links into two columns for better layout */}
              <div className="col col--6">
                <h4 className="footer-heading">Curriculum</h4>
                <ul className="footer-list">
                  {footerLinks.slice(0, 2).map((link, idx) => (
                    <li key={idx} className="footer-list-item">
                      <Link to={link.to} className="footer-link">
                        {link.title}
                      </Link>
                    </li>
                  ))}
                </ul>
              </div>
              <div className="col col--6">
                <h4 className="footer-heading">&nbsp;</h4> {/* Spacer for alignment */}
                <ul className="footer-list">
                  {footerLinks.slice(2, 4).map((link, idx) => (
                    <li key={idx} className="footer-list-item">
                      <Link to={link.to} className="footer-link">
                        {link.title}
                      </Link>
                    </li>
                  ))}
                </ul>
              </div>
            </div>
          </div>
        </div>
        
        <div className="footer-bottom margin-top--xl">
          <div className="footer-divider margin-bottom--md" />
          <div className="text--center">
            Copyright © {currentYear} {siteConfig.title}. Built by A_.
          </div>
        </div>
      </div>
    </footer>
  );
}

export default React.memo(Footer);
