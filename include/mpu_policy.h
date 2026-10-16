#ifndef MPU_POLICY_H
#define MPU_POLICY_H

/* Production sampling switch, independent from diagnostic telemetry.
 * Compatibility: old RC1 experiment builds define MPU_FRESHNESS_AB_B.
 * These compile to exactly the same skip/retry policy. */
#if defined(MPU_FRESHNESS_AB_B) && !defined(MPU_FRESHNESS_DIAG)
#error "Legacy MPU_FRESHNESS_AB_B requires MPU_FRESHNESS_DIAG"
#endif
#if defined(MPU_FRESHNESS_AB_B) && !defined(MPU_SAMPLING_POLICY_B)
#define MPU_SAMPLING_POLICY_B 1
#endif

#define MPU_POLICY_B_RETRY_DELAY_US 220U
#define MPU_POLICY_B_RETRY_START_MAX_US 600U
#define MPU_POLICY_B_RETRY_END_MAX_US 850U

#endif /* MPU_POLICY_H */
