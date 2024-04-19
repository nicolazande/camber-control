/**
 * \file synchronization-lib.c
 *
 * \brief Synchronization API + example of usage.
 *
 */

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include <stdio.h>
#include <math.h>


/***********************************************************************************************************************
 * Defines
 **********************************************************************************************************************/
#define TSPP 0.01 // path planner sampling time [s] <-- NOTE: TSPP = ECU cycle time (don't change this)
#define VELOCITY_OFFSET 0U //velocity offset for hysteresis
#define MM_TO_DEG_SCALING 15 //scaling factor to compare error in mm with error in deg for EccB
#define MAX_PPA_SI 3.2297e+03 // max acceleration on load side [m/s^2] <-- NOTE: 50000 = profile acceleration in OD [rpm/s]
#define MAX_PPV_SI 155.6722 // max velocity on load side [m/s] <-- NOTE: 5800 = profile velocity in OD [rpm]
#define KPPPH_SI 32.2971 //gain scheduler high value
#define KPPPL_SI 3.2297 //gain scheduler low value
#define HPP 12.5 //gain scheduler exponential decay
#define SYNCH_TARGET_RATIO 1 //balance between synchronization force and steering force


/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
/**
 * \brief Convert angular position in [deg] to linear position in [mm] for rear axis.
 *
 * \param [in]  x   \ref Position in degrees.
 *
 * \return Position in millimiters.
 */
double deg2m_r(double x)
{
    /* taylor expansion coefficients */
    double c5 = 0.0000000000624;
    double c4 = -0.000000000045;
    double c3 = -0.0000037025199;
    double c2 = -0.0001050951019;
    double c1 = 0.0716813513211;
    double c0 = 0.0041785649983;

    /* 
     * Linear actuator position in meters. Maybe you can find a simpler
     * map for this like sin(x), I don't know the mechanichs of the car.
     */
    x = x * (x * (x * (x * (c5 * x + c4) + c3) + c2) + c1) + c0;

    return x;
}

/**
 * \brief Convert angular position in [deg] to linear position in [mm] for front axis.
 *
 * \param [in]  x   \ref Position in degrees.
 *
 * \return Position in millimiters.
 */
double deg2mm_f(double x)
{
    /* taylor expansion coefficients */
    double c5 = 0.0000000000703;
    double c4 = 0.0000000031969;
    double c3 = -0.000004402643;
    double c2 = -0.0001583845823;
    double c1 = 0.083336057797;
    double c0 = 0.0046001004738;

    /* 
     * Linear actuator position in meters. Maybe you can find a simpler
     * map for this like sin(x), I don't know the mechanichs of the car.
     */
    x = x * (x * (x * (x * (c5 * x + c4) + c3) + c2) + c1) + c0;

    return x;
}

/**
 * \brief Path planner gain scheduler.
 *
 * \param [in]  error      	\ref Error between target and simulated path planner position.
 *
 * \return Adaptime path planner proportional gain.
 */
double gain_scheduler(double error)
{
    /* 
     * Compute optimal proportional gain according to position error.
     * NOTE: 1 / (1 + x) is a good approximation of exp(-x)for x > 0.
     */
    return KPPPH_SI + (KPPPL_SI - KPPPH_SI) * (1 / (1 + HPP * fabs(error)));
}

/**
 * \brief Path planner.
 * 
 * \param [inout]  target_position   	  \ref Requested smoothed position.
 * \param [in]     feedback_position   	  \ref Slow dynamics actuator feedback.
 * \param [in]     delay               	  \ref Error between fastest and slowest actuator.
 */
void path_planner(double *target_position, double feedback_position, double delay)
{
    /* static variables (needed for memory, use pointers otherwise) */
    static double position_old = 0;
    static double velocity_old = 0;

    /* pointer check */
    if (target_position)
    {
        /*
         * Current position error (target vs feedback). The selected feedback
         * should be always the one of the actuator with the slowest dynamics.
         * In this way the path planner adaptive control action is limited by
         * it (this avoids big oscillations).
         */
        double error = *target_position - feedback_position;

        /**
         * Run gain scheduler (adaptive control).
         * NOTE: you can also use a constant gain but this will not
         *       prevent overshoot.
         */
        double Km = gain_scheduler(error);

        /* velocity (path planner system input) */
        double velocity = error * Km;

        /* 
         * Synchronization correction, it always opposes to the steering force,
         * in case the commanded velocity is zero (the slow actuator reached the
         * target), don't apply the correction, the other actuators will recover
         * quickly and reduce the gap.
         */
        if (velocity > VELOCITY_OFFSET)
        {
            /** TODO: increase SYNCH_TARGET_RATIO if the synchronization is slow */
            velocity = velocity - SYNCH_TARGET_RATIO*Km*delay;
        }
        else if (velocity < -VELOCITY_OFFSET)
        {
            /** TODO: increase SYNCH_TARGET_RATIO if the synchronization is slow */
            velocity = velocity + SYNCH_TARGET_RATIO*Km*delay;
        }
    
        /* get runtime velocity boundaries (consider max acceleration) */
        double velocity_high = fmin(velocity_old + MAX_PPA_SI * TSPP, MAX_PPV_SI);
        double velocity_low = fmax(velocity_old - MAX_PPA_SI * TSPP, -MAX_PPV_SI);
        velocity = fmin(fmax(velocity, velocity_low), velocity_high);

        /* 
         * Forward kinematics simulation for position (speed integral). Also the
         * real position feedback of the slowest actuator could be used in order
         * to increase the reactivity of the path planner, however this introduces
         * oscillations and jumps in the commanded velocity that consequently
         * cause oscillations on the actual position and so on.
         */
        *target_position = position_old + TSPP * velocity;

        /* save position and velocity (for the next cycle) */
        position_old = *target_position;
        velocity_old = velocity;
    }
}



/** example of usage */
int main()
{
    /* position feedback signals */
    double LinB_feedback_mm; // = ...
    double EccB_feedback_deg; // = ...
    /** TODO: read feadback position of LinB (in mm) and EccB (in deg) through EtherCAT */

    /*
     * Since for the moment we have only angular to linear map, the setpoint
     * needs to be given in degrees (at load side).
     */
    double target_position;
    /*
     * Feedback position of the actuator with slow dyanamics. In this case
     * it is EccB. Also in this case if a linear actuator is slower we'll
     * need the linear to angular map.
     */
    double feedback_position = EccB_feedback_deg;

    /* 
     * Delay between first and last actuators. For the same reason (missing
     * linear to angular map), it has to be computed in mm and than scaled
     * up in order to be comparable with the degree target position, the
     * controller will compensate for the mismatch. In case you want to
     * synchronize more than two actuators you can implement the same
     * logic shown in the simulink model, i.e. find the signal with the
     * min and max difference wrt the target position, compute the
     * difference, take the absolute value and scale it up.
     */
    double delay = MM_TO_DEG_SCALING * fabs(LinB_feedback_mm - deg2m_r(EccB_feedback_deg));

    
    /* ...logic .... */


    /* update reference actuator target position with path planner */
    path_planner(&target_position, feedback_position, delay);

    /* EccB target position is equal to target_position i ndeg in this case */
    double EccB_position = target_position;

    /* LinB target position is equal to target_position converted in mm */
    double LinB_position = deg2m_r(target_position);


    /* ...logic .... */


    return 0;
}