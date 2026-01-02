import React, { useState } from 'react';
import styles from './RobotControls.module.css';

/**
 * RobotControls Component
 * Provides interactive controls for robot simulation
 * Includes teleoperation, joint control, and navigation commands
 */
const RobotControls = ({
  robotName = 'Robotic Assistant',
  controlMode = 'teleop', // teleop, joint, nav
  onCommand = null,
  title = 'Robot Controls',
  description = 'Interactive controls for robot teleoperation and navigation',
  showStatus = true
}) => {
  const [currentMode, setCurrentMode] = useState(controlMode);
  const [velocity, setVelocity] = useState({ linear: 0, angular: 0 });
  const [jointPositions, setJointPositions] = useState({
    joint1: 0,
    joint2: 0,
    joint3: 0,
    joint4: 0,
    joint5: 0,
    joint6: 0
  });
  const [navigationGoal, setNavigationGoal] = useState({ x: 0, y: 0, theta: 0 });
  const [isConnected, setIsConnected] = useState(true);
  const [robotStatus, setRobotStatus] = useState('idle');
  const [batteryLevel, setBatteryLevel] = useState(85);

  const handleMove = (command) => {
    if (onCommand) {
      onCommand(command);
    }

    // Update local state for visualization
    switch(command) {
      case 'forward':
        setVelocity(prev => ({ ...prev, linear: Math.min(prev.linear + 0.1, 1.0) }));
        setRobotStatus('moving_forward');
        break;
      case 'backward':
        setVelocity(prev => ({ ...prev, linear: Math.max(prev.linear - 0.1, -1.0) }));
        setRobotStatus('moving_backward');
        break;
      case 'left':
        setVelocity(prev => ({ ...prev, angular: Math.min(prev.angular + 0.1, 1.0) }));
        setRobotStatus('turning_left');
        break;
      case 'right':
        setVelocity(prev => ({ ...prev, angular: Math.max(prev.angular - 0.1, -1.0) }));
        setRobotStatus('turning_right');
        break;
      case 'stop':
        setVelocity({ linear: 0, angular: 0 });
        setRobotStatus('idle');
        break;
    }
  };

  const handleJointChange = (joint, value) => {
    setJointPositions(prev => ({
      ...prev,
      [joint]: parseFloat(value)
    }));
  };

  const handleNavGoalChange = (axis, value) => {
    setNavigationGoal(prev => ({
      ...prev,
      [axis]: parseFloat(value)
    }));
  };

  const sendNavigationGoal = () => {
    if (onCommand) {
      onCommand({
        type: 'nav_goal',
        goal: navigationGoal
      });
    }
    setRobotStatus('navigating');
  };

  const toggleConnection = () => {
    setIsConnected(!isConnected);
    setRobotStatus(isConnected ? 'disconnected' : 'connected');
  };

  const getRobotStatusColor = () => {
    switch(robotStatus) {
      case 'idle': return '#28a745';
      case 'moving_forward': return '#007bff';
      case 'moving_backward': return '#007bff';
      case 'turning_left': return '#007bff';
      case 'turning_right': return '#007bff';
      case 'navigating': return '#ffc107';
      case 'disconnected': return '#dc3545';
      default: return '#6c757d';
    }
  };

  return (
    <div className={styles.robotControlsContainer}>
      <div className={styles.controlsHeader}>
        <div className={styles.controlsTitle}>
          <span className={styles.controlsIcon}>🤖</span>
          <h3>{title}</h3>
          <span className={styles.robotName}>{robotName}</span>
        </div>
        <p className={styles.controlsDescription}>{description}</p>
      </div>

      <div className={styles.controlsContent}>
        <div className={styles.modeSelector}>
          <button
            className={`${styles.modeButton} ${currentMode === 'teleop' ? styles.activeMode : ''}`}
            onClick={() => setCurrentMode('teleop')}
          >
            🕹️ Teleoperation
          </button>
          <button
            className={`${styles.modeButton} ${currentMode === 'joint' ? styles.activeMode : ''}`}
            onClick={() => setCurrentMode('joint')}
          >
            ⚙️ Joint Control
          </button>
          <button
            className={`${styles.modeButton} ${currentMode === 'nav' ? styles.activeMode : ''}`}
            onClick={() => setCurrentMode('nav')}
          >
            🧭 Navigation
          </button>
        </div>

        {currentMode === 'teleop' && (
          <div className={styles.teleopControls}>
            <div className={styles.teleopPad}>
              <button
                className={styles.teleopButton}
                onMouseDown={() => handleMove('forward')}
                onMouseUp={() => handleMove('stop')}
                onTouchStart={() => handleMove('forward')}
                onTouchEnd={() => handleMove('stop')}
              >
                ⬆️
              </button>
              <div className={styles.teleopRow}>
                <button
                  className={styles.teleopButton}
                  onMouseDown={() => handleMove('left')}
                  onMouseUp={() => handleMove('stop')}
                  onTouchStart={() => handleMove('left')}
                  onTouchEnd={() => handleMove('stop')}
                >
                  ⬅️
                </button>
                <button
                  className={styles.teleopButton}
                  onClick={() => handleMove('stop')}
                >
                  ⬛
                </button>
                <button
                  className={styles.teleopButton}
                  onMouseDown={() => handleMove('right')}
                  onMouseUp={() => handleMove('stop')}
                  onTouchStart={() => handleMove('right')}
                  onTouchEnd={() => handleMove('stop')}
                >
                  ➡️
                </button>
              </div>
              <button
                className={styles.teleopButton}
                onMouseDown={() => handleMove('backward')}
                onMouseUp={() => handleMove('stop')}
                onTouchStart={() => handleMove('backward')}
                onTouchEnd={() => handleMove('stop')}
              >
                ⬇️
              </button>
            </div>
            <div className={styles.velocityDisplay}>
              <p>Linear: {velocity.linear.toFixed(2)}</p>
              <p>Angular: {velocity.angular.toFixed(2)}</p>
            </div>
          </div>
        )}

        {currentMode === 'joint' && (
          <div className={styles.jointControls}>
            {Object.keys(jointPositions).map((joint) => (
              <div key={joint} className={styles.jointControl}>
                <label>{joint.replace('joint', 'Joint ')}: {jointPositions[joint].toFixed(2)} rad</label>
                <input
                  type="range"
                  min="-3.14"
                  max="3.14"
                  step="0.01"
                  value={jointPositions[joint]}
                  onChange={(e) => handleJointChange(joint, e.target.value)}
                  className={styles.jointSlider}
                />
                <div className={styles.jointValue}>{jointPositions[joint].toFixed(2)}</div>
              </div>
            ))}
          </div>
        )}

        {currentMode === 'nav' && (
          <div className={styles.navControls}>
            <div className={styles.navInput}>
              <label>X Position: {navigationGoal.x.toFixed(2)} m</label>
              <input
                type="range"
                min="-10"
                max="10"
                step="0.1"
                value={navigationGoal.x}
                onChange={(e) => handleNavGoalChange('x', e.target.value)}
              />
            </div>
            <div className={styles.navInput}>
              <label>Y Position: {navigationGoal.y.toFixed(2)} m</label>
              <input
                type="range"
                min="-10"
                max="10"
                step="0.1"
                value={navigationGoal.y}
                onChange={(e) => handleNavGoalChange('y', e.target.value)}
              />
            </div>
            <div className={styles.navInput}>
              <label>Orientation: {navigationGoal.theta.toFixed(2)} rad</label>
              <input
                type="range"
                min="-3.14"
                max="3.14"
                step="0.01"
                value={navigationGoal.theta}
                onChange={(e) => handleNavGoalChange('theta', e.target.value)}
              />
            </div>
            <button
              onClick={sendNavigationGoal}
              className={styles.navButton}
            >
              🚀 Send Goal
            </button>
          </div>
        )}
      </div>

      {showStatus && (
        <div className={styles.statusPanel}>
          <div className={styles.statusRow}>
            <div className={styles.statusItem}>
              <span className={styles.statusLabel}>Status:</span>
              <span
                className={styles.statusValue}
                style={{ color: getRobotStatusColor() }}
              >
                {robotStatus}
              </span>
            </div>
            <div className={styles.statusItem}>
              <span className={styles.statusLabel}>Battery:</span>
              <span className={styles.statusValue}>{batteryLevel}%</span>
            </div>
            <div className={styles.statusItem}>
              <span className={styles.statusLabel}>Connection:</span>
              <span className={styles.statusValue}>
                {isConnected ? '🟢 Connected' : '🔴 Disconnected'}
              </span>
            </div>
          </div>
          <button
            onClick={toggleConnection}
            className={`${styles.statusButton} ${isConnected ? styles.disconnectButton : styles.connectButton}`}
          >
            {isConnected ? '🔌 Disconnect' : '🔌 Connect'}
          </button>
        </div>
      )}

      <div className={styles.controlsInfo}>
        <details className={styles.controlsDetails}>
          <summary>Control Information</summary>
          <div className={styles.controlsDetailsContent}>
            <p><strong>Control Mode:</strong> {currentMode}</p>
            <p><strong>Robot:</strong> {robotName}</p>
            <p><strong>Max Velocity:</strong> 1.0 m/s (linear), 1.0 rad/s (angular)</p>
            <p><strong>Joint Range:</strong> ±π radians</p>
            <p><strong>Note:</strong> This is a simulation interface. Real robot control requires proper ROS 2 setup.</p>
          </div>
        </details>
      </div>
    </div>
  );
};

export default RobotControls;