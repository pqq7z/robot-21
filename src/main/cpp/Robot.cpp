/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cameraserver/CameraServer.h>

#include <frc/livewindow/LiveWindow.h>

#define SIZEOF_ARRAY(array_name) (sizeof(array_name) / sizeof(array_name[0]))

typedef struct
{
    double y;
    double z;
    double t;
    double intake;
    double discharge;
} move_step_t;

typedef struct
{
    move_step_t *steps;
    size_t total_steps;
} move_seq_t;

// back away
move_step_t mv_default[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0.0},
  // drive
  {0.4, 0.0, 1.5, 0.0, 0.0},
  {0.2, 0.0, 1.3, 0.0, 0.0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0.0},
};

// discharge and back away
move_step_t mv_2[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0.0},
  // discharge
  {0.0, 0.0, 3.0, 0.0, -0.5},
  // drive
  {-0.4, 0.0, 1.5, 0.0, 0.0},
  {-0.2, 0.0, 1.0, 0.0, 0.0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0.0},
};

// drive, discharge and back away
move_step_t mv_3[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0.0},
  // drive forward
  {0.4, 0.0, 1.2, 0.0, 0.0},
  {0.2, 0.0, 1.0, 0.0, 0.0},
  // discharge
  {0.0, 0.0, 3.0, 0.0, -0.5},
  // back away
  // {-0.2, 0.3, 2.0, 0.0, 0.0},
  // {-0.2, 0.0, 1.0, 0.0, 0.0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0.0},
};

move_step_t mv_4[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0.0},
  // drive
  {-0.4, 0.0, 2.2, 0.0, 0.0},
  {0.0, 0.0, 1.0, 0.0, 0.0},
  {0.0, 0.2, 2.0, 0.0, 0.0},
  // stop
  {0.0, 0.0, 0.5, 0.0, 0.0},
};
move_step_t mv_5[] =
{
  // delay
  {0.0, 0.0, 0.0, 0.0, 0.0},
  // drive
  {0.4, 0.0, 1.5, 0.0, 0.0},

};
move_step_t mv_6[] = 
{
  {-0.4, 0.0, 1.0, 0.0, 0.0}, //forward 5 feet
  {0.0, -0.2, 2.0, 0.0,0.0},  //90 degrees left
  {-0.4, 0.0, 1.0, 0.0, 0.0}, // forward 5 feet
  {0.0, 0.2, 2.0, 0.0, 0.0},  //90 degrees right
  {-0.4, 0.0, 3.2, 0.0, 0.0}, //15 feet forward
  {0.0, 0.2, 2.0, 0.0, 0.0},  //90 degrees right
  {-0.4, 0.0, 1.0, 0.0, 0.0}, //5 feet forward
  {-0.4, -0.2, 2.0, 0.0, 0.0}, //90 degrees left
  {-0.4, 0.0, 1.0, 0.0, 0.0}, //5 feet forward
  {0.0, -0.2, 2.0, 0.0, 0.0}, //90 degrees left
  {-0.4, 0.0, 1.0, 0.0, 0.0}, // 5 feet forward
  {-0.0, 0.2, 2.0, 0.0, 0.0}, //90 degrees left
  {-0.4, 0.0, 1.0, 0.0, 0.0}, // 5 feet forward
  {0.0, 0.2, 2.0, 0.0, 0.0},  // 90 degrees right
  {-0.4, 0.0, 3.2, 0.0, 0.0}, //15 feet forwards
  {0.0, 0.2, 2.0, 0.0, 0.0},  // 90 degrees right
  {-0.4, 0.0, 1.0, 0.0, 0.0}, // 5 feet forward
  {0.0, -0.2, 2.0, 0.0, 0.0}, // 90 degrees left
  {-0.4, 0.0, 1.0, 0.0, 0.0}, // 5 feet forward
};

move_step_t none[] =
{
	{0.0, 0.0, 0.0, 0.0, 0.0},
};

move_seq_t mv = {none, 0};

double accel_max = 0.02;
#if 1
// static 
void clamp(double &value, const double ul, const double ll)
{
    if (value < ll)
	{
    	value = ll;
	}
    else if (value > ul)
	{
    	value = ul;
	}
}
#endif
#if 1
static 
void scale(double &value, const double deadband, const double ll, const double ul)
{
//printf("1: value=%5.2f\n", value);

  // adjust for deadband
  if (value > deadband) value -= deadband;
  else if (value < -deadband) value += deadband;
  else value = 0;

//printf("2: value=%5.2f\n", value);

  // scale based output range / input range
  value *= ((ul - ll) / (1.0 - deadband));

//printf("3: value=%5.2f\n", value);

  // offset to minimum
  if (value > 0) value += ll;
  else value -= ll;

//printf("4: value=%5.2f\n", value);
}
#endif


class Robot : public frc::TimedRobot {
 public:
  Robot() {
    printf("robot-21 v1.1.0 %s %s\n", __DATE__, __TIME__);

    m_robotDrive.SetExpiration(0.1);
    m_timer.Start();
    
#if 1
    frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
#endif

    frc::SmartDashboard::PutNumber("delay", 3);
    frc::SmartDashboard::PutNumber("auto", 1);
    frc::SmartDashboard::PutNumber("t", 0);
    frc::SmartDashboard::PutNumber("y", 0);
    frc::SmartDashboard::PutNumber("z", 0);
    frc::SmartDashboard::PutNumber("Shooter",0);
  }

  void AutonomousInit() override {
    // m_timer.Reset();
    // m_timer.Start();

    double delay = frc::SmartDashboard::GetNumber("delay", 5);
    double move = frc::SmartDashboard::GetNumber("auto", 1);
    double t = frc::SmartDashboard::GetNumber("t", 0);
    double y = frc::SmartDashboard::GetNumber("y", 0);
    double z = frc::SmartDashboard::GetNumber("z", 0);
    y=y/10;
    z=z/10;

    mv.steps = mv_default;
    mv.total_steps = SIZEOF_ARRAY(mv_default);

    if (move == 2)
    {
      mv.steps = mv_2;
      mv.total_steps = SIZEOF_ARRAY(mv_2);
    }
    else if (move == 3)
    {
      mv.steps = mv_3;
      mv.total_steps = SIZEOF_ARRAY(mv_3);
    }
    else if (move == 4)
    {
      mv.steps = mv_4;
      mv.total_steps = SIZEOF_ARRAY(mv_4);
    }
    else if (move == 5)
    {
      mv.steps = mv_5;
      mv.total_steps = SIZEOF_ARRAY(mv_5);
      mv.steps[1].y = y; 
      mv.steps[1].t = t; 
      mv.steps[1].z = z; 
    }
    else if (move == 6)
    {
      mv.steps = mv_6;
      mv.total_steps = SIZEOF_ARRAY(mv_6);
    }

    t_step_end = m_timer.Get() + delay;
    // t_step_end = m_timer.Get() + mv.steps[0].t;
    step = 0;

    move_complete = false;

    printf("%d: y=%5.2f z=%5.2f t=%5.2f i=%5.2f d=%5.2f\n", step+1,
        mv.steps[0].y, mv.steps[0].z, mv.steps[0].t, mv.steps[0].intake, mv.steps[0].discharge);
  }

  void AutonomousPeriodic() override 
  {
    double tmp_y = last_y;
    double tmp_z = last_z;

    double t = m_timer.Get();

    if (step < mv.total_steps)
    {
        step_complete = t > t_step_end;

        if (step_complete)
        {
          step += 1;
            if (step < mv.total_steps)
            {
                t_step_end += mv.steps[step].t;

                printf("%d: y=%5.2f z=%5.2f t=%5.2f i=%5.2f d=%5.2f\n", step+1,
                    mv.steps[step].y, mv.steps[step].z, mv.steps[step].t, mv.steps[step].intake, mv.steps[step].discharge);
            }
            else
            {
                // sequence complete
              printf("%5.2f: move complete\n", t);

              step = mv.total_steps;
              move_complete = true;
            }
        }
    }

    double y = 0.0;
    double z = 0.0;

    if (step < mv.total_steps)
    {
        y = mv.steps[step].y;
        z = mv.steps[step].z;
    }

    if (y == 0.0)
    {
        // set sign for pure rotations
        // z *= rotation;
    }

    // limit acceleration
    double dy = y - last_y;
    double dz = z - last_z;

    clamp(dy, accel_max, -accel_max);
    clamp(dz, accel_max, -accel_max);

    last_y = last_y + dy;
    last_z = last_z + dz;

    if (tmp_y != last_y || tmp_z != last_z)
    {
        printf("%5.2f: y: %5.2f => %5.2f / x: %5.2f => %5.2f\n", t, y, last_y, z, last_z);
    }

    m_robotDrive.ArcadeDrive(-last_y, last_z, false);

    m_lift.Set(mv.steps[step].intake);

    m_discharge.Set(mv.steps[step].discharge);
  }

  void TeleopInit() override 
  {
     m_shooter = frc::SmartDashboard::GetNumber("Shooter", 5);
     m_shooter = m_shooter / 10;

     printf("shooter=%5.2f\n", m_shooter);
  }

  void TeleopPeriodic() override 
  {
    // driver controls
    double y = m_stickd.GetRawAxis(1);
    double z = m_stickd.GetRawAxis(4);

    scale(y, 0.15, .2, .7);
    scale(z, 0.15, .2, .6);

    m_robotDrive.ArcadeDrive(y, z);

    // operator controls
    if (m_sticko.GetRawButton(4))
      m_telescope.Set(m_shooter);
    else if (m_sticko.GetRawButton(1))
      m_telescope.Set(-m_shooter);
    else 
      m_telescope.Set(0);

    if (m_sticko.GetRawButton(3))
      m_lift.Set(-0.8);
    else 
      m_lift.Set(0);

    if (m_sticko.GetRawButton(6))
      m_winch.Set(0.6);
    else if (m_sticko.GetRawButton(5))
      m_winch.Set(-0.6);
    else 
      m_winch.Set(0);

    if (m_sticko.GetRawButton(2))
      m_discharge.Set(-0.8);
    else 
      m_discharge.Set(0);
  } 

  void TestPeriodic() override {}

 private: 
  // Robot drive system
  frc::PWMVictorSPX m_right{0};
  frc::PWMVictorSPX m_left{1};
  frc::PWMVictorSPX m_telescope{5};
  frc::PWMVictorSPX m_lift{6};
  frc::PWMVictorSPX m_winch{3};
  frc::PWMVictorSPX m_discharge{4};

  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  frc::Joystick m_stickd{0};
  frc::Joystick m_sticko{1};
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer m_timer;
  
  double m_timestamp {0};
  
  size_t total_steps;
  size_t step;
  double t_step_end;
  
	bool move_complete;
  bool step_complete;
  
  double last_z {0};
  double last_y {0};

  double m_shooter {0};
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

