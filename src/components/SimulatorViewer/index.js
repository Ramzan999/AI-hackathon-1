import React, { useState, useEffect } from 'react';
import styles from './SimulatorViewer.module.css';

/**
 * SimulatorViewer Component
 * Provides an embedded viewer for robotics simulations
 * Can display Gazebo, Isaac Sim, or Unity simulation content
 */
const SimulatorViewer = ({
  simulatorType = 'gazebo',
  simulationUrl = '',
  title = 'Robotics Simulation Viewer',
  description = 'Interactive simulation environment',
  controls = true,
  autoStart = true
}) => {
  const [isLoaded, setIsLoaded] = useState(false);
  const [isPlaying, setIsPlaying] = useState(autoStart);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (autoStart) {
      setIsPlaying(true);
    }
  }, [autoStart]);

  const handlePlayPause = () => {
    setIsPlaying(!isPlaying);
  };

  const handleReset = () => {
    setIsLoaded(false);
    setTimeout(() => setIsLoaded(true), 100);
  };

  const getSimulatorIcon = () => {
    switch(simulatorType.toLowerCase()) {
      case 'gazebo':
        return '🤖';
      case 'isaac sim':
        return '⚡';
      case 'unity':
        return '🎮';
      default:
        return '🎮';
    }
  };

  const getSimulatorColor = () => {
    switch(simulatorType.toLowerCase()) {
      case 'gazebo':
        return '#4CAF50';
      case 'isaac sim':
        return '#FF9800';
      case 'unity':
        return '#00BCD4';
      default:
        return '#607D8B';
    }
  };

  return (
    <div className={styles.simulatorContainer}>
      <div className={styles.simulatorHeader}>
        <div className={styles.simulatorTitle}>
          <span className={styles.simulatorIcon}>{getSimulatorIcon()}</span>
          <h3>{title}</h3>
          <span
            className={styles.simulatorBadge}
            style={{ backgroundColor: getSimulatorColor() }}
          >
            {simulatorType}
          </span>
        </div>
        <p className={styles.simulatorDescription}>{description}</p>
      </div>

      <div className={styles.simulatorContent}>
        {error ? (
          <div className={styles.errorContainer}>
            <p>Error loading simulation: {error.message}</p>
            <button onClick={handleReset} className={styles.resetButton}>
              Reset Simulation
            </button>
          </div>
        ) : (
          <div className={styles.simulationFrame}>
            {simulationUrl ? (
              <iframe
                src={simulationUrl}
                className={styles.simulationIframe}
                title={title}
                onLoad={() => setIsLoaded(true)}
                onError={(e) => setError(e)}
                allowFullScreen
              />
            ) : (
              <div className={styles.placeholder}>
                <div className={styles.placeholderIcon}>{getSimulatorIcon()}</div>
                <p>Simulation environment would appear here</p>
                <p className={styles.placeholderSubtext}>
                  Configure with a simulation URL or use local simulation
                </p>
              </div>
            )}
          </div>
        )}
      </div>

      {controls && (
        <div className={styles.simulatorControls}>
          <button
            onClick={handlePlayPause}
            className={`${styles.controlButton} ${styles.playButton}`}
            disabled={!!error}
          >
            {isPlaying ? '⏸️ Pause' : '▶️ Play'}
          </button>
          <button
            onClick={handleReset}
            className={`${styles.controlButton} ${styles.resetButton}`}
          >
            🔄 Reset
          </button>
          <div className={styles.simulationStatus}>
            <span className={`${styles.statusIndicator} ${isPlaying ? styles.statusActive : styles.statusInactive}`}>
              {isPlaying ? '● Active' : '○ Idle'}
            </span>
          </div>
        </div>
      )}

      <div className={styles.simulatorInfo}>
        <details className={styles.simulatorDetails}>
          <summary>Simulation Information</summary>
          <div className={styles.simulatorDetailsContent}>
            <p><strong>Simulator:</strong> {simulatorType}</p>
            <p><strong>Status:</strong> {isLoaded ? 'Loaded' : 'Loading...'} {isPlaying ? '(Running)' : '(Paused)'}</p>
            <p><strong>Controls:</strong> {controls ? 'Enabled' : 'Disabled'}</p>
          </div>
        </details>
      </div>
    </div>
  );
};

export default SimulatorViewer;