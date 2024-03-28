#pragma once

#include <cstdlib>
#include <algorithm>

#include "IO/pin.hpp"

#include "stm32g4xx_hal_tim.h"

using namespace PIN_BOARD;

namespace StepperMotor{

    struct StepperCfg
    {
        PIN<PinWriteable> step_pin;
        PIN<PinWriteable> direction_pin;
        PIN<PinWriteable> sleep_pin;
        TIM_HandleTypeDef* htim {};
        uint32_t timChannel {0x00000000U};
        uint32_t criticalNofSteps {};
        bool directionInverted {};
        bool shake_scan_enabled_ {};
        float A {};
        float Vmax {};
        float Vmin {};
    };

    enum MOTOR_EVENT {
        EVENT_NULL = 0,
        EVENT_STOP,
        EVENT_CSS,  //	constant speed reached
        EVENT_CSE   //  constant speed end
    };

    enum Mode
    {
        IDLE,
        ACCEL,
        CONST,
        DECCEL,
        in_ERROR
    };

    enum class Direction{
        BACKWARDS = 0,
        FORWARD = 1
    };

    struct StepperTask{
        float acceleration;
        float start_speed;
        float max_speed;
        Direction dir;
        int steps;
        bool pending = false;
        [[nodiscard]] constexpr bool isPending() const{
            return pending;
        }
        constexpr void setPending(){
            pending = true;
        }
        constexpr void unsetPending(){
            pending = false;
        }
    };

    class StepperMotorBase{
    public:
        using MOTOR_IOS = PIN<PinWriteable>;

        StepperMotorBase() = delete;
        const StepperMotorBase& operator=(const StepperMotorBase &) = delete;
        StepperMotorBase& operator=(StepperMotorBase &) = delete;
        StepperMotorBase(StepperMotorBase&) = delete;
        StepperMotorBase(StepperMotorBase&&)= delete;

        void MotorRefresh(){
            if(mode_ == IDLE || mode_ == Mode::in_ERROR)
                return;
            CalcSpeed_();
            CalcRegValue_();
            ImplCorrection();
        }

        virtual void ImplCorrection() = 0;

        void SetMotorTask(StepperTask&& task)
        {
            currentTask_ = task;
            currentTask_.setPending();
            if(motorMoving_)
                mode_ = DECCEL;
            else
                ExecPendingTask_();
        }

        void StopMotor(){
            if(motorMoving_){
                HAL_TIM_PWM_Stop_IT(htim_, timChannel_);
//                sleep_pin_.setValue(HIGH);
                motorMoving_ = false;
                mode_ = Mode::IDLE;
                event_ = EVENT_STOP;
            }
        }

        void ChangeDirection(){
            SetDirection_(static_cast<bool>(currentDirection_) ? Direction::BACKWARDS : Direction::FORWARD);
            V_ = Vmin_;
            mode_ = Mode::ACCEL;
            CalcRegValue_();
            currentStep_ = 0;
            accel_step_ = 0;
        }

        [[nodiscard]] bool IsMotorMoving() const {
            return motorMoving_;
        }

        [[nodiscard]] Mode GetMode() const {
            return mode_;
        }

        [[nodiscard]] MOTOR_EVENT GetEvent() const {
            return event_;
        }

        [[nodiscard]] Direction GetCurrentDirection() const {
            return currentDirection_;
        }
    protected:
        explicit StepperMotorBase(StepperMotor::StepperCfg& cfg)
            : step_pin_(cfg.step_pin),
              direction_pin_(cfg.direction_pin),
              sleep_pin_(cfg.sleep_pin),
              timChannel_(cfg.timChannel),
              htim_(cfg.htim),
              directionInverted_(cfg.directionInverted),
              Vmin_(cfg.Vmin)
        {
            kCriticalNofSteps_ = cfg.criticalNofSteps;
            timerFreq_ = SystemCoreClock / (cfg.htim->Instance->PSC);
        };

        MOTOR_IOS step_pin_;
        MOTOR_IOS direction_pin_;
        MOTOR_IOS sleep_pin_;

        TIM_HandleTypeDef *htim_;
        uint32_t timChannel_;
        uint32_t timerFreq_;

        inline static int kCriticalNofSteps_ {0};
        int steps_to_go_ {0};

        int currentStep_ {0};
        int accel_step_ {0};

        float A_ {0.0f};
        float V_ {0.0f};
        float Vmin_ {0.0f};
        float Vmax_ {0.0f};

        StepperTask currentTask_;
        inline static Direction currentDirection_ {Direction::FORWARD};
        Mode mode_ {Mode::IDLE};
        MOTOR_EVENT event_ {EVENT_STOP};

        bool directionInverted_ {false};
        bool motorMoving_ {false};

        void StartMotor_(uint32_t steps){
            if(!motorMoving_){
                accel_step_ = 0;
                currentStep_ = 0;
                mode_ = Mode::ACCEL;
                motorMoving_ = true;
                steps_to_go_ = steps;
                sleep_pin_.setValue(LOW);
                CalcRegValue_();
                HAL_TIM_PWM_Start_IT(htim_, timChannel_);
            }
        }

        void CalcRegValue_(){
            if(V_ > 0){
                uint32_t tick_count = timerFreq_ / uint32_t(V_);
                if(tick_count > 0 && tick_count < UINT16_MAX){
                    __HAL_TIM_SET_AUTORELOAD(htim_, tick_count);
                    __HAL_TIM_SET_COMPARE(htim_, timChannel_, tick_count / 2);
                }
            }
        }

        void SetDirection_(Direction newDirection){
            currentDirection_ = newDirection;
            if(directionInverted_) direction_pin_.setValue( static_cast<bool>(currentDirection_) ?
                            LOGIC_LEVEL(Direction::BACKWARDS) : LOGIC_LEVEL(Direction::FORWARD) );
            else
                direction_pin_.setValue(LOGIC_LEVEL(currentDirection_));
        }

        void ExecPendingTask_(){
            currentTask_.unsetPending();
            StopMotor();
            A_ = currentTask_.acceleration;
            Vmin_ = currentTask_.start_speed;
            Vmax_ = currentTask_.max_speed;
            V_ = currentTask_.start_speed;
            SetDirection_(currentTask_.dir);
            StartMotor_(currentTask_.steps ? currentTask_.steps : kCriticalNofSteps_);
        }

        void CalcSpeed_(){
            if (mode_ == Mode::IDLE) return;
            switch (mode_)
            {
                case Mode::ACCEL:
                {
                    if (V_ >= Vmax_)
                    {
                        V_ = Vmax_;
                        event_ = EVENT_CSS;
                        mode_ = Mode::CONST;
                    }else
                        V_ += A_;

                    if (accel_step_ >= steps_to_go_ / 2)
                    {
                        mode_ = Mode::DECCEL;
                        break;
                    }
                    accel_step_++;
                }
                break;

                case Mode::CONST:
                {
                    if(currentStep_ + accel_step_ >= steps_to_go_)
                        mode_ = Mode::DECCEL;
                }
                break;

                case Mode::DECCEL:
                {
                    if(accel_step_ > 0){
                        V_ -= A_;
                        if (V_ < Vmin_) V_ = Vmin_;
                        accel_step_--;
                    }else if(currentTask_.isPending())
                        ExecPendingTask_();
                }
                break;

                default:
                    break;
            }

            if (mode_ == Mode::ACCEL || mode_ == Mode::CONST || mode_ == Mode::DECCEL)
                currentStep_++;
        }
    };

} //namespace StepperMotor