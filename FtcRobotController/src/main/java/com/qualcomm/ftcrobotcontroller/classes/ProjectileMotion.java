package com.qualcomm.ftcrobotcontroller.classes;

public class ProjectileMotion
{
    //const declarations
    private final double InToCm = 2.54;
    private final double DegToRad = 3.1415926 / 180.0;

    //input variables
    private double VelocityInitial; // cm/s^2
    private double DistanceToVortex; // cm

    //measurements of fields, subject to change
    private final double BallRadius = 1.875 * InToCm;  // cm
    private final double BallStartingHeight = 6.0 * InToCm;  // cm
    private final double VortexRadius = 10.375 * InToCm;  // cm
    private final double VortexPoleHeight = 48.0 * InToCm;  // cm

    //intermediate variables
    private double HorizontalVelocityInitial;  // cm/s^2
    private double VerticalVelocityInitial;  // cm/s^2

    //output variables
    private double AngleMin, AngleMax; // radians

    //constructors
    public ProjectileMotion()
    {
        DistanceToVortex = 0.0;
        VelocityInitial = 0.0;
    }

    public ProjectileMotion(double Distance, double Velocity)
    {
        DistanceToVortex = Distance; // cm
        VelocityInitial = Velocity; // cm/s^2
    }

    //input subroutines
    public void InputDistance(double Distance)
    {
        DistanceToVortex = Distance; //cm
    }

    public void InputVelocity(double Velocity)
    {
        VelocityInitial = Velocity;
    }

    //return functions
    public double GetAngleFreedom()
    {
        //return AngleMax - AngleMin; // radians
        return (AngleMax - AngleMin)/DegToRad; // degrees
    }

    public double GetAngleMin()
    {
        //return AngleMin; // radians
        return AngleMin/DegToRad; // degrees
    }

    public double GetAngleMax()
    {
        //return AngleMax; // radians
        return AngleMax/DegToRad; // degrees
    }

    //private functions

    //setting variables for TestProjectile Calculations
    private void SolveParametricInitialVelocities(double Angle)
    {
        HorizontalVelocityInitial = Math.cos(Angle) * VelocityInitial;
        VerticalVelocityInitial = Math.sin(Angle) * VelocityInitial;
    }

    //Y value at a specified time
    private double GetYBallValue(double Angle, double Time)
    {
        return -490.0 * Time * Time + VerticalVelocityInitial * Time + BallStartingHeight; // cm
    }

    //using linear bisection method to find Angle Min, decreasing run time for Angle Min to logarithmic; basically binary search
    private double LinearBisectionAngleMin(double Angle1, double Angle2)
    {
        double AngleAvg = (Angle1 + Angle2)/2.0;
        if(Angle2 - Angle1 <= 0.0005 && Angle2 - Angle1 >= -0.0005)
        { //if within acceptable margin of error, stop recursion
            return AngleAvg;
        }
        SolveParametricInitialVelocities(AngleAvg);
        if(AllTestBallProjectile(AngleAvg))
        {
            return LinearBisectionAngleMin(Angle1, AngleAvg);
        }
        else
        {
            return LinearBisectionAngleMin(AngleAvg, Angle2);
        }
    }

    //Tests for projectile
    private boolean AllTestBallProjectile(double Angle)
    { //condensed test for projectile
        return (TestBallProjectile1(Angle) && TestBallProjectile2(Angle));
    }

    private boolean TestBallProjectile1(double Angle)
    {
        //Ball goes over first pole
        return TestBallProjectileBase(Angle, DistanceToVortex - VortexRadius, VortexPoleHeight, false);
    }

    private boolean TestBallProjectile2(double Angle)
    {
        //Ball goes under second pole
        return TestBallProjectileBase(Angle, DistanceToVortex + VortexRadius, VortexPoleHeight, true);
    }

    private boolean TestBallProjectileBase(double Angle, double HorizontalDistanceToObject, double ObjectHeight, boolean HigherOrLower)
    {
        double TimeToObject = HorizontalDistanceToObject / HorizontalVelocityInitial; // Time it takes to travel to object, assuming 0 drag
        double YValueAtObjectPosition = GetYBallValue(Angle, TimeToObject);           // Y value of ball at object
        if(HigherOrLower)
        {                                                            // Boolean test for whether you want to test higher or lower
            return YValueAtObjectPosition <= ObjectHeight;
        }
        else{
            return YValueAtObjectPosition >= ObjectHeight;
        }
    }

    //main angle calculations
    private void UpdateTrajectoryCalculations()
    {
        double theta;
        // trying to find AngleMax, since it is usually close to about 70 degrees
        for(theta = Math.PI/2; theta > 0.0; theta -= 0.01)
        { // projectile might actually not pass this if test but is still viable,
            SolveParametricInitialVelocities(theta);        // but would only have a < .6 degrees range
            if(AllTestBallProjectile(theta))
            {
                break;
            }
        }
        //testing if Projectile actually meets requirements
        if(theta >= 0.1)
        {
            for(theta=theta; theta <= Math.PI/2; theta += 0.0005)
            { //making theta more accurate
                SolveParametricInitialVelocities(theta);
                if(!AllTestBallProjectile(theta))
                {
                    break;
                }
            }
            AngleMax = theta;
            AngleMin = LinearBisectionAngleMin(0.0, AngleMax - 0.005); //using Linear Bisection Method to find AngleMin
        }
    }

	/*
	//main function for testing
	public static void main(String[] args)
	{
		ProjectileMotion test1 = new ProjectileMotion(206.35,700.0);
		test1.UpdateTrajectoryCalculations();
		System.out.println(test1.GetAngleMax());
		System.out.println(test1.GetAngleMin());
		System.out.println(test1.GetAngleFreedom());
	}*/
}