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
    void togglePlay();

    void update(float delta_time);

    // Animation progress (0.0 to 1.0)
    float getProgress() const;

    bool isPlaying() const;
    bool isFinished() const;

    void setLooping(bool looping);
    bool isLooping() const;
    void toggleLooping();

private:
    float current_time_;
    float duration_;
    float speed_;
    bool playing_;
    bool loop_;
};

#endif // ANIMATION_HPP
