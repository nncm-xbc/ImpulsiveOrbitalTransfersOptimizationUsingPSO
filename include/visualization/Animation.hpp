/**
 * @file Animation.hpp
 * @brief Animation controller for orbital transfer visualization
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides time-based animation controls for visualizing orbital transfers
 * and spacecraft trajectories. Supports play/pause, speed control, and
 * looping functionality.
 */

#ifndef ANIMATION_HPP
#define ANIMATION_HPP

/**
 * @class Animation
 * @brief Controls animation timing and playback for orbital visualizations
 *
 * Manages animation state and timing for orbital transfer visualizations.
 * Features include:
 * - Play/pause control
 * - Variable speed adjustment
 * - Looping support
 * - Progress tracking (0.0 to 1.0)
 * - Real-time updates based on frame delta time
 *
 * The animation system is designed to smoothly animate orbital transfers,
 * allowing users to visualize spacecraft motion along transfer trajectories
 * at controllable speeds.
 */
class Animation {
public:
    /**
     * @brief Constructor with default duration
     * @param duration Total animation duration in seconds (default 5.0)
     *
     * Creates an animation controller with specified duration.
     * Animation starts in paused state and can be controlled via
     * user input or programmatic calls.
     */
    Animation(float duration = 5.0f);

    /**
     * @brief Start animation playback
     *
     * Begins or resumes animation from current position.
     * If animation was finished and not looping, resets to beginning.
     */
    void start();

    /**
     * @brief Pause animation playback
     *
     * Suspends animation at current position. Can be resumed
     * with start() call without losing current progress.
     */
    void pause();

    /**
     * @brief Reset animation to beginning
     *
     * Sets current time back to 0.0, effectively restarting
     * the animation sequence. Does not change play/pause state.
     */
    void reset();

    /**
     * @brief Set animation playback speed multiplier
     * @param speed Speed multiplier (1.0 = normal, 2.0 = double speed, 0.5 = half speed)
     *
     * Controls how fast the animation plays relative to real time.
     * Values > 1.0 speed up animation, values < 1.0 slow it down.
     * Negative values are not supported.
     */
    void setSpeed(float speed);

    /**
     * @brief Set total animation duration
     * @param duration New duration in seconds
     *
     * Changes the total time for one complete animation cycle.
     * If current time exceeds new duration, progress may jump to end.
     */
    void setDuration(float duration);

    /**
     * @brief Toggle between play and pause states
     *
     * Convenience method that starts animation if paused,
     * or pauses animation if playing.
     */
    void togglePlay();

    /**
     * @brief Update animation state based on elapsed time
     * @param delta_time Time elapsed since last update (seconds)
     *
     * Advances animation time if playing, taking into account
     * speed multiplier. Handles looping behavior when animation
     * reaches the end.
     */
    void update(float delta_time);

    /**
     * @brief Get current animation progress
     * @return Progress value from 0.0 (start) to 1.0 (end)
     *
     * Returns normalized progress through the animation.
     * Can be used to interpolate positions along transfer trajectories
     * or control other time-dependent visualization parameters.
     */
    float getProgress() const;

    /**
     * @brief Check if animation is currently playing
     * @return true if animation is playing, false if paused
     */
    bool isPlaying() const;

    /**
     * @brief Check if animation has reached the end
     * @return true if current time >= duration
     *
     * For looping animations, this briefly returns true at the
     * end of each cycle before resetting.
     */
    bool isFinished() const;

    /**
     * @brief Enable or disable animation looping
     * @param looping true to enable looping, false to stop at end
     *
     * When looping is enabled, animation automatically restarts
     * from the beginning when it reaches the end. When disabled,
     * animation stops and can be manually reset.
     */
    void setLooping(bool looping);

    /**
     * @brief Check if looping is enabled
     * @return true if animation will loop, false otherwise
     */
    bool isLooping() const;

    /**
     * @brief Toggle looping behavior
     *
     * Convenience method that enables looping if disabled,
     * or disables looping if enabled.
     */
    void toggleLooping();

private:
    /** @brief Current animation time in seconds */
    float current_time_;

    /** @brief Total animation duration in seconds */
    float duration_;

    /** @brief Speed multiplier for playback rate */
    float speed_;

    /** @brief Whether animation is currently playing */
    bool playing_;

    /** @brief Whether animation should loop when finished */
    bool loop_;
};

#endif // ANIMATION_HPP
