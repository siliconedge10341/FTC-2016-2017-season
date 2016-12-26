package com.qualcomm.ftcrobotcontroller.classes;
public class ProjectileMotion
{
    //const declarations
    private final double InToCm = 2.54;
    private final double DegToRad = 3.1415926 / 180.0;
    private final double ShooterAngle = 78 * DegToRad;
    private final double ShooterDeviation = 3 * DegToRad;

    //input variables
    private double VelocityInitial; // cm/s^2
    private double DistanceToVortex; // cm

    //measurements of fields, subject to change
    private final double BallRadius = 1.875 * InToCm;  // cm
    private final double BallStartingHeight = 6.0 * InToCm;  // cm
    private final double VortexRadius = 10.35 * InToCm;  // cm
    private final double VortexPoleHeight = 46.7 * InToCm;  // cm

    //intermediate variables
    private double HorizontalVelocityInitial;  // cm/s^2
    private double VerticalVelocityInitial;  // cm/s^2

    //output variables
    private double AngleMin, AngleMax; // radians

    //secondary output variables
    private double VelocityMin, VelocityMax; // cm/s^2

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
        DistanceToVortex = Distance; // cm
    }

    public void InputVelocity(double Velocity)
    {
        VelocityInitial = Velocity; // cm/s^2
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

    //functions

    //setting variables for TestProjectile Calculations
    private void SolveParametricInitialVelocities(double Angle, double Velocity)
    {
        HorizontalVelocityInitial = Math.cos(Angle) * Velocity;
        VerticalVelocityInitial = Math.sin(Angle) * Velocity;
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
        if(Angle2 - Angle1 <= 0.0005 && Angle2 - Angle1 >= -0.00005)
        { //if within acceptable margin of error, stop recursion
            return AngleAvg;
        }
        SolveParametricInitialVelocities(AngleAvg, VelocityInitial);
        if(AllTestBallProjectile(AngleAvg))
        {
            return LinearBisectionAngleMin(Angle1, AngleAvg);
        }
        else
        {
            return LinearBisectionAngleMin(AngleAvg, Angle2);
        }
    }

    //using linear bisection method to find VelocityMax, decreasing run time for VelocityMax to logarithmic; basically binary search
    private double LinearBisectionVelocityMax(double Velocity1, double Velocity2)
    {
        double VelocityAvg = (Velocity1 + Velocity2)/2.0;
        if(Velocity2 - Velocity1 <= 0.5 && Velocity2 - Velocity1 >= -0.5)
        { //if within acceptable margin of error, stop recursion
            return VelocityAvg;
        }
        SolveParametricInitialVelocities(ShooterAngle, VelocityAvg);
        if(AllTestBallProjectile(ShooterAngle))
        {
            return LinearBisectionVelocityMax(VelocityAvg, Velocity2);
        }
        else
        {
            return LinearBisectionVelocityMax(Velocity1, VelocityAvg);
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
    public void UpdateTrajectoryCalculations()
    {
        double theta;
        // trying to find AngleMax, since it is usually close to about 70 degrees
        for(theta = Math.PI/2; theta > 0.0; theta -= 0.01)
        { // projectile might actually not pass this if test but is still viable,
            SolveParametricInitialVelocities(theta, VelocityInitial);        // but would only have a < .6 degrees range
            if(AllTestBallProjectile(theta))
            {
                break;
            }
        }
        //testing if Projectile actually meets requirements
        if(theta >= 0.1)
        {
            for(theta=theta; theta <= Math.PI/2; theta += 0.00005)
            { //making theta more accurate
                SolveParametricInitialVelocities(theta, VelocityInitial);
                if(!AllTestBallProjectile(theta))
                {
                    break;
                }
            }
            AngleMax = theta;
            AngleMin = LinearBisectionAngleMin(0.0, AngleMax - 0.005); //using Linear Bisection Method to find AngleMin
        }
        else //unable to shoot under conditions
        {
            AngleMax = 0;
            AngleMin = 0;
        }
    }

    //velocity calculations
    public void UpdatePowerRequirement()
    {
        //replace the constant angle with the angle from the other subroutine if necessary
        double Velocity;
        for(Velocity = 400; Velocity < 701; Velocity += 10)
        {
            SolveParametricInitialVelocities(ShooterAngle, Velocity);
            if(AllTestBallProjectile(ShooterAngle))
            {
                break;
            }
        }
        if(Velocity < 700)
        {
            VelocityMin = Velocity;
            for(Velocity = VelocityMin; Velocity > 400; Velocity += 0.5)
            {
                SolveParametricInitialVelocities(ShooterAngle, Velocity);
                if(!AllTestBallProjectile(ShooterAngle))
                {
                    break;
                }
            }
            VelocityMin = Velocity;
            VelocityMax = LinearBisectionVelocityMax(VelocityMin + 10, 800);
        }
        else // unable to shoot under conditions
        {
            VelocityMax = 0;
            VelocityMax = 0;
        }
    }

    //main function for testing
    public static void main(String[] args)
    {
        //first test main
		/*ProjectileMotion test1 = new ProjectileMotion(206.35,700.0);
		test1.UpdateTrajectoryCalculations();
		System.out.println(test1.GetAngleMax());
		System.out.println(test1.GetAngleMin());
		System.out.println(test1.GetAngleFreedom());*/

        //second test main
		/*
		ProjectileMotion TestProjectileMotion = new ProjectileMotion();
		Scanner scanner = new Scanner(System.in);
		double OptimumVelocity = 0.0;
		double LargestAngleFreedom = 0.0;
		double TempDoubleVar = 0.0;
		double Distance = 0.0;
		System.out.print("Enter distance: ");
		Distance = scanner.nextDouble();
		while(Distance > 10.0)
		{
			OptimumVelocity = 0.0;
			LargestAngleFreedom = 0.0;
			TempDoubleVar = 0.0;
			TestProjectileMotion.InputDistance(Distance);
			for(double Velocity = 400.0; Velocity < 1000.0; Velocity += 10.0)
			{
				TestProjectileMotion.InputVelocity(Velocity);
				TestProjectileMotion.UpdateTrajectoryCalculations();
				TempDoubleVar = TestProjectileMotion.GetAngleFreedom();
				if(TempDoubleVar > LargestAngleFreedom)
				{
					OptimumVelocity = Velocity;
					LargestAngleFreedom = TestProjectileMotion.GetAngleFreedom();
				}
			}
			OptimumVelocity += 10.0;
			TestProjectileMotion.InputVelocity(OptimumVelocity);
			TestProjectileMotion.UpdateTrajectoryCalculations();
			LargestAngleFreedom = TestProjectileMotion.GetAngleFreedom();
			for(double Velocity = OptimumVelocity; Velocity > 400.0; Velocity -= 0.05)
			{
				TestProjectileMotion.InputVelocity(Velocity);
				TestProjectileMotion.UpdateTrajectoryCalculations();
				TempDoubleVar = TestProjectileMotion.GetAngleFreedom();
				if(TempDoubleVar > LargestAngleFreedom)
				{
					OptimumVelocity = Velocity;
					LargestAngleFreedom = TestProjectileMotion.GetAngleFreedom();
				}
			}
			System.out.println("The optimum velocity is: " + OptimumVelocity);
			TestProjectileMotion.InputVelocity(OptimumVelocity);
			TestProjectileMotion.UpdateTrajectoryCalculations();
			System.out.println("The maximum angle is: " + TestProjectileMotion.GetAngleMax());
			System.out.println("The minimum angle is: " + TestProjectileMotion.GetAngleMin());
			System.out.println("The angle freedom is: " + TestProjectileMotion.GetAngleFreedom());
			System.out.print("Enter distance: ");
			Distance = scanner.nextDouble();
		}
		*/

        //third test main
    }
}