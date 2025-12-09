import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '285'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '0e4'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '02f'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'b51'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', 'f1c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', 'a9b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '40b'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'a7e'),
    routes: [
      {
        path: '/docs/intro/foundations',
        component: ComponentCreator('/docs/intro/foundations', '528'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/intro/setup-edge-kit',
        component: ComponentCreator('/docs/intro/setup-edge-kit', '9b5'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/intro/setup-workstation',
        component: ComponentCreator('/docs/intro/setup-workstation', '872'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module1/architecture-concepts',
        component: ComponentCreator('/docs/module1/architecture-concepts', 'bc9'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module1/packages-rclpy',
        component: ComponentCreator('/docs/module1/packages-rclpy', '87d'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module1/urdf-rviz2',
        component: ComponentCreator('/docs/module1/urdf-rviz2', 'be8'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module2/gazebo-setup',
        component: ComponentCreator('/docs/module2/gazebo-setup', 'e51'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module2/physics-collision',
        component: ComponentCreator('/docs/module2/physics-collision', '3ef'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module2/sensor-simulation',
        component: ComponentCreator('/docs/module2/sensor-simulation', 'ef9'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module3/isaac-sim-intro',
        component: ComponentCreator('/docs/module3/isaac-sim-intro', '183'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module3/nav2-humanoids',
        component: ComponentCreator('/docs/module3/nav2-humanoids', '4f0'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module3/vslam-perception',
        component: ComponentCreator('/docs/module3/vslam-perception', 'd14'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module4/capstone-synthesis',
        component: ComponentCreator('/docs/module4/capstone-synthesis', '223'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module4/cognitive-planning',
        component: ComponentCreator('/docs/module4/cognitive-planning', 'b32'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/module4/conversational-robotics',
        component: ComponentCreator('/docs/module4/conversational-robotics', '02e'),
        exact: true,
        sidebar: "tutorialSidebar"
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '347'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
