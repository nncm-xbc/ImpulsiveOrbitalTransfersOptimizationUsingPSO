#include "Animation.hpp"

#include <cmath>

Animation::Animation(float duration) :
    duration_(duration),
    current_time_(0.0f),
    playing_(false),
    speed_(1.0f),
    loop_(true) {}

void Animation::start() { playing_ = true; }
void Animation::pause() { playing_ = false; }
void Animation::reset() { current_time_ = 0.0f; }
void Animation::togglePlay() { playing_ = !playing_; }
void Animation::setLooping(bool looping) { loop_ = looping; }
void Animation::toggleLooping() { loop_ = !loop_; }

void Animation::setSpeed(float speed) { speed_ = speed; }
void Animation::setDuration(float duration) { duration_ = duration; }

void Animation::update(float delta_time) {
    if (playing_) {
        current_time_ += delta_time * speed_;
        if (current_time_ > duration_) {
            if (loop_) {
                current_time_ = 0.0f;
            } else {
                current_time_ = duration_;
                playing_ = false; // Stop at the end
            }
        }
    }
}

float Animation::getProgress() const {
    return current_time_ / duration_;
}

bool Animation::isPlaying() const { return playing_; }
bool Animation::isFinished() const { return current_time_ >= duration_; }
bool Animation::isLooping() const { return loop_; }
