#include <gtest/gtest.h>
#include <fstream>
#include <iostream>

std::fstream fout;

double getProfileSpeed(double t, double speed_tar)
{
  static double acc_out = 0;
  static double vel_out = 0;
  static double prev_t = 0;
  const double acc_max = 1.6; 
  const double acc_step = 0.2;
  const double dec_max = 1; 
  const double dec_step = 0.1;
  const double vel_tor = 0.1;
  double dt = t-prev_t;
  prev_t = t;
  if (abs(speed_tar - vel_out) > vel_tor) 
  {
    if (speed_tar > vel_out) 
    {
      acc_out = std::min(acc_out + acc_step, acc_max);
    } else {
      acc_out = std::max(acc_out - dec_step, -dec_max);
    }
    vel_out += acc_out*dt;
  }
  if (abs(speed_tar - vel_out) <= vel_tor)
  {
    vel_out = speed_tar;
    acc_out = 0;
  }
  fout << t << "," << speed_tar << "," << acc_out << "," << vel_out << std::endl;
}

TEST (scurve, SCurveTest)
{
  const std::vector<double> speed_inputs = {0, 1, 2, 3, 0, 1, 2, 3, -2, -3, 3, 0};
	double t = 0;
  fout.open("/home/justin/trapezoidal_profile_test.csv", std::ios::out);
  fout << "t,tar_speed,acc,vel_out\n";
	for (int i = 0; i < speed_inputs.size(); i++)
	{
		double speed_tar = speed_inputs[i];
		for (int j = 0; j < 100; j++) // steps per velocity
		{
			t += 0.1;
			double speed_out = getProfileSpeed(t, speed_tar);
		}
	}
  fout.close();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  int test = RUN_ALL_TESTS();
  return test;
}