#include <gtest/gtest.h>
#include <fstream>
#include <iostream>

std::fstream fout;

double getSCurveSpeed(double t, double speed_tar)
{
	static double acc_out = 0;
	static double vel_out = 0;
	static double prev_t = 0;
	static int jerk = 0;
	const double acc_max = 2; 
	const double acc_step = 0.1;
	const double appr_tar_vel = 0.1;
	const double reached_tar_vel = 0.05;
  double dt = t-prev_t;
  prev_t = t;
  if (abs(speed_tar-vel_out) < reached_tar_vel)
	{
		jerk = 0;
	} else if (abs(speed_tar-vel_out) < appr_tar_vel) 
	{
		if (abs(speed_tar) > abs(vel_out)) // acc
		{
			jerk = -1;
		} else 
		{
			jerk = 1;
		}
	} else if (abs(speed_tar-vel_out) >= appr_tar_vel) 
  {
    if (abs(vel_out) < appr_tar_vel)
    {
      jerk = 1;
    } else
    {
      jerk = 0;
    }
  }
	acc_out += (jerk*acc_step);
	if (acc_out > acc_max)
	{
		acc_out = (jerk*acc_max);
		jerk = 0;
	}
  vel_out += acc_out*dt;
  if (speed_tar >= 0 && vel_out > speed_tar) {vel_out = speed_tar; }
  if (speed_tar < 0 && vel_out < speed_tar) {vel_out = speed_tar; }
	fout << t << "," << speed_tar << "," 
       << jerk << "," << acc_out << "," << vel_out << std::endl;
}

TEST (scurve, SCurveTest)
{
  const std::vector<double> speed_inputs = {0, 1, 2, 3, 0, 1, 2, 3, -2, -3, 3, 0};
	double t = 0;
  fout.open("/home/justin/Scurve_test.csv", std::ios::out);
  fout << "t,tar_speed,jerk,acc,vel_out\n";
	for (int i = 0; i < speed_inputs.size(); i++)
	{
		double speed_tar = speed_inputs[i];
		for (int j = 0; j < 100; j++) // steps per velocity
		{
			t += 0.1;
			double speed_out = getSCurveSpeed(t, speed_tar);
		}
	}
  fout.close();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  int test = RUN_ALL_TESTS();
  return test;
}