#include <gtrack.h>
#include <gtrack_int.h>
typedef enum
{
    TRACKING_DEFAULT_PARAM_SET = 0,
    TRACKING_TRAFFIC_MONITORING_PARAM_SET,
    TRACKING_PEOPLE_COUNTING_PARAM_SET
} TRACKING_ADVANCED_PARAM_SET;

typedef enum
{
    TRACKING_PARAM_SET_TM = 0,
    TRACKING_PARAM_SET_PC
} TRACKING_ADVANCED_PARAM_SET_TABLE;

/* This test application (traffic monitoring), wants to modify default parameters */
GTRACK_sceneryParams appSceneryParamTable[2] = {
    1, {{-7.f, 15.f, 10.f, 185.f}, {0.f, 0.f, 0.f, 0.f}}, 1, {{-6.f, 14.f, 15.f, 60.f}, {0.f, 0.f, 0.f, 0.f}}, /* boundary box: {-1,12,15,75}, static box {0,11,19,50}, both in (left x, right x, bottom y, top y) format */
    0,
    {{0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}},
    0,
    {{0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}} /* no boundary boxes, static boxes */
};
GTRACK_gatingParams appGatingParamTable[2] = {
    {16.f, {12.f, 8.f, 0.f}}, /* TM: 16 gating volume, Limits are set to 8m in length, 2m in width, no limit in doppler */
    {2.f, {2.f, 2.f, 0.f}}    /* PC: 2 gating volume, Limits are set to 2m in length, 2m in width, no limit in doppler */
};
GTRACK_stateParams appStateParamTable[2] = {
    {3U, 3U, 5U, 1000U, 5U},  /* TM: 3 frames to activate, 3 to forget, 5 to delete irrespective */
    {10U, 5U, 10U, 1000U, 5U} /* PC: det2act, det2free, act2free, stat2free, exit2free */
};
GTRACK_allocationParams appAllocationParamTable[2] = {
    {250.f, 100.f, 1.f, 5U, 2.8f, 2.f}, /* TM: any SNRs, 1m/s minimal velocity, 3 points with 4m in distance, 2m/c in velocity  separation */
    {150.f, 250.f, 0.1f, 5U, 1.f, 2.f}  /* PC: SNRs 150 (250 obscured), 0.1 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
};
/* Using standard deviation of uniformly distributed variable in the range [a b]: 1/sqrt(12)*(b-a) */
GTRACK_varParams appVariationParamTable[2] = {
    /* Standard deviation of uniformly distributed number in range [a b]: sqrt(1/12)*(b-a) */
    {4.f / 3.46f, 1.5f / 3.46f, 1.f}, /* TM: 1m height, 1m in width, 2 m/s for doppler */
    {1.f / 3.46f, 1.f / 3.46f, 1.f}   /* PC: 1m height, 1m in width, 1 m/s for doppler */
};

GTRACK_targetDesc targetDescr[20];

int main(int argc, char *argv[])
{

    int n = 0;
    GTRACK_moduleConfig config;
    GTRACK_advancedParameters advParams;

    TRACKING_ADVANCED_PARAM_SET trackingParamSet;
    trackingParamSet = TRACKING_DEFAULT_PARAM_SET;

    switch (trackingParamSet)
    {
    case TRACKING_DEFAULT_PARAM_SET:
        // Do not configure advanced parameters, use library default parameters
        config.advParams = 0;
        break;

    case TRACKING_TRAFFIC_MONITORING_PARAM_SET:
        /* Initialize CLI configuration: */
        memset((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
        advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_TM];
        advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_TM];
        advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_TM];
        advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_TM];
        advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_TM];

        config.advParams = &advParams;
        config.initialRadialVelocity = -8; // for TM, detected targets are approaching
        config.maxAcceleration[0] = 2;     // for PC, maximum acceleration in lateral direction is set to 5m/s2
        config.maxAcceleration[1] = 20;    // for PC, maximum acceleration is longitudinal direction set to 5m/s2
        config.maxAcceleration[2] = 5;
        break;

    case TRACKING_PEOPLE_COUNTING_PARAM_SET:

        /* Initialize CLI configuration: */
        memset((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
        advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_PC];
        advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_PC];
        advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_PC];
        advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_PC];
        advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_PC];

        config.advParams = &advParams;
        config.initialRadialVelocity = 0; //For PC, detected target velocity is unknown
        config.maxAcceleration[0] = 5;    // for PC, maximum acceleration in lateral direction is set to 5m/s2
        config.maxAcceleration[1] = 20;   // for PC, maximum acceleration is longitudinal direction set to 5m/s2
        config.maxAcceleration[2] = 5;
        break;

    default:
        return -1;
    }

    config.stateVectorType = GTRACK_STATE_VECTORS_2DA; // Track two dimensions with acceleration
    config.verbose = GTRACK_VERBOSE_NONE;
    config.maxNumPoints = (uint16_t)250;
    config.maxNumTracks = (uint16_t)20;
    config.maxRadialVelocity = (float)0.1f;
    config.radialVelocityResolution = (float)0.039; //currently hardcoded, get from cfg in the future
    config.deltaT = (float)50 * 0.001f;
    void *h;
    int32_t errCode;

    h = gtrack_create(&config, &errCode);
    float cart[2] = {0, 0};
    float *sph;
    gtrack_cartesian2spherical(GTRACK_STATE_VECTORS_2DA, cart, sph);
    printf("%p\n", h);

    return 0;
}