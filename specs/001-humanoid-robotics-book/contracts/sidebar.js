/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation
 */

const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction & Setup',
      items: [
        'intro/foundations',
        'intro/setup-workstation',
        'intro/setup-edge-kit',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module1/architecture-concepts',
        'module1/packages-rclpy',
        'module1/urdf-rviz2',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Sim)',
      items: [
        'module2/gazebo-setup',
        'module2/physics-collision',
        'module2/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (Isaac)',
      items: [
        'module3/isaac-sim-intro',
        'module3/vslam-perception',
        'module3/nav2-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & Capstone',
      items: [
        'module4/conversational-robotics',
        'module4/cognitive-planning',
        'module4/capstone-synthesis',
      ],
    },
  ],
};

module.exports = sidebars;
