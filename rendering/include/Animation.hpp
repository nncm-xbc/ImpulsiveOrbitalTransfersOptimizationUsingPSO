#ifndef ANIMATION_HPP
#define ANIMATION_HPP

class Animation {
public:
    Animation(float duration = 5.0f);

    // Controls
    void start();
    void pause();
    void reset();
    void setSpeed(float speed);
    void setDuration(float duration);

    void update(float delta_time);

    // Animation progress (0.0 to 1.0)
    float getProgress() const;

    bool isPlaying() const;
    bool isFinished() const;

private:
    float current_time_;
    float duration_;
    float speed_;
    bool playing_;
};

#endif // ANIMATION_HPP