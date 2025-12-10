import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '9d8'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '70e'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '3f1'),
            routes: [
              {
                path: '/docs/intro/foundations',
                component: ComponentCreator('/docs/intro/foundations', 'ff2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro/setup-edge-kit',
                component: ComponentCreator('/docs/intro/setup-edge-kit', 'de5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro/setup-workstation',
                component: ComponentCreator('/docs/intro/setup-workstation', '651'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/architecture-concepts',
                component: ComponentCreator('/docs/module1/architecture-concepts', '3a0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/packages-rclpy',
                component: ComponentCreator('/docs/module1/packages-rclpy', 'f20'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/urdf-rviz2',
                component: ComponentCreator('/docs/module1/urdf-rviz2', 'a14'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/gazebo-setup',
                component: ComponentCreator('/docs/module2/gazebo-setup', '21b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/physics-collision',
                component: ComponentCreator('/docs/module2/physics-collision', '682'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/sensor-simulation',
                component: ComponentCreator('/docs/module2/sensor-simulation', '0f3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac-sim-intro',
                component: ComponentCreator('/docs/module3/isaac-sim-intro', '284'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/nav2-humanoids',
                component: ComponentCreator('/docs/module3/nav2-humanoids', 'aac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/vslam-perception',
                component: ComponentCreator('/docs/module3/vslam-perception', 'a5a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/capstone-synthesis',
                component: ComponentCreator('/docs/module4/capstone-synthesis', 'b3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/cognitive-planning',
                component: ComponentCreator('/docs/module4/cognitive-planning', '646'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/conversational-robotics',
                component: ComponentCreator('/docs/module4/conversational-robotics', 'cce'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
