# V15.16 刚体惯量 Engineering V1

> 状态：**ENGINEERING_V1**。这不是实测惯量，也不是高保真辨识动力学。

- total mass: `3.4515 kg`
- Mass authority: `V15_15_实测质量账本_v1.json`
- COM authority: `V15_15_COM账本_v2.json`
- collision proxy / deployment / gravity: `NO / NO / NO`

## link2

- mass: `0.7615 kg`
- COM (link m): `[-0.09682602754229415, -0.01627177564337673, 0.00026852096508443425]`
- confidence: `ENGINEERING_V1`
- limitations: `['CANONICAL_PRINT_GEOMETRY_REUSED_AS_REQUIRED; GEOMETRY_CENTROID_NOT_RECOMPUTED; SMALL_INTERPART_INTERPENETRATION_REMAINS_BELOW_1_PERCENT', 'canonical_geometry_is_working_copy_repair', 'gear_ratio_reflected_inertia_not_added', 'material_density_distribution_assumed_uniform', 'motor_internal_rotor_dynamic_inertia_not_added', 'not_vendor_measured_subassembly_inertia']`
- inertia tensor (kg·m²):

```text
[0.0011506145878779188, -0.00095403936341770246, 9.4047158306883176e-06]
[-0.00095403936341770246, 0.0072175587272901437, 6.2574216239836676e-06]
[9.4047158306883176e-06, 6.2574216239836676e-06, 0.0074077910960128933]
```

## link3

- mass: `1.09 kg`
- COM (link m): `[-0.10115006408445788, 0.034091146941368534, 0.02591627514565488]`
- confidence: `ENGINEERING_V1`
- limitations: `['BOUNDED_OCCUPIED_VOLUME_BUT_EMPIRICAL_NONCONSERVATIVE_ISOTROPIC_SIMILAR_SHAPE_SECOND_MOMENT_ESTIMATE_AFTER_NORMAL_OCCT_WORKER_FAILURE', 'PROTECTED_V2_INTRINSIC_REUSED; GEOMETRY_CENTROID_NOT_RECOMPUTED', 'canonical_geometry_is_working_copy_repair', 'gear_ratio_reflected_inertia_not_added', 'material_density_distribution_assumed_uniform', 'motor_internal_rotor_dynamic_inertia_not_added', 'not_vendor_measured_subassembly_inertia', 'uniform_proxy_geometry_not_measured_mass_distribution', 'wiring_geometry_incomplete']`
- inertia tensor (kg·m²):

```text
[0.0014953205051949686, -3.4642987466909751e-06, 0.0027501824929620057]
[-3.4642987466909751e-06, 0.012662694095746066, 2.5701148702950441e-06]
[0.0027501824929620057, 2.5701148702950441e-06, 0.011454987201389681]
```

## link4

- mass: `0.675 kg`
- COM (link m): `[-0.003256288062399695, 0.026691901325969727, 0.07712273077803503]`
- confidence: `ENGINEERING_V1`
- limitations: `['BOUNDED_OCCUPIED_VOLUME_BUT_EMPIRICAL_NONCONSERVATIVE_ISOTROPIC_SIMILAR_SHAPE_SECOND_MOMENT_ESTIMATE_AFTER_NORMAL_OCCT_WORKER_FAILURE', 'PROTECTED_V2_INTRINSIC_REUSED; GEOMETRY_CENTROID_NOT_RECOMPUTED', 'RESIDUAL_SPATIAL_INERTIA_MODEL_UNCERTAINTY', 'canonical_geometry_is_working_copy_repair', 'gear_ratio_reflected_inertia_not_added', 'material_density_distribution_assumed_uniform', 'motor_internal_rotor_dynamic_inertia_not_added', 'not_vendor_measured_subassembly_inertia', 'uniform_proxy_geometry_not_measured_mass_distribution', 'wiring_geometry_incomplete']`
- inertia tensor (kg·m²):

```text
[0.0017846928666986044, 5.5761918086767607e-06, 3.2605406086893188e-05]
[5.5761918086767607e-06, 0.0014691530030288575, -0.00026299080742248982]
[3.2605406086893188e-05, -0.00026299080742248982, 0.0004889558170829342]
```

## link5

- mass: `0.504 kg`
- COM (link m): `[-0.007314793455186482, -0.0001338266798615075, 0.06388991222545329]`
- confidence: `ENGINEERING_V1`
- limitations: `['RESIDUAL_SPATIAL_INERTIA_MODEL_UNCERTAINTY', 'gear_ratio_reflected_inertia_not_added', 'motor_internal_rotor_dynamic_inertia_not_added', 'not_vendor_measured_subassembly_inertia', 'uniform_proxy_geometry_not_measured_mass_distribution', 'wiring_geometry_incomplete']`
- inertia tensor (kg·m²):

```text
[0.00069235493231530379, -3.2307499399953101e-07, 0.00011373297010931771]
[-3.2307499399953101e-07, 0.00070380925687268687, 7.9503308686525174e-07]
[0.00011373297010931771, 7.9503308686525174e-07, 0.00030027903217345458]
```

## link6

- mass: `0.125 kg`
- COM (link m): `[-1.1047636330058231e-15, 1.360430323081585e-16, -0.0005011957785718855]`
- confidence: `ENGINEERING_V1`
- limitations: `['gear_ratio_reflected_inertia_not_added', 'motor_internal_rotor_dynamic_inertia_not_added', 'not_vendor_measured_subassembly_inertia']`
- inertia tensor (kg·m²):

```text
[1.1518299692789527e-05, 8.6467750457995536e-20, 1.7766902612207907e-18]
[8.6467750457995536e-20, 1.1518299692790563e-05, -5.3195234386365257e-21]
[1.7766902612207907e-18, -5.3195234386365257e-21, 2.3015838872973936e-05]
```

## gripper

- mass: `0.296 kg`
- COM (link m): `[-0.0034049824167213345, -0.0007997027279568269, 0.03493651236222258]`
- confidence: `ENGINEERING_V1`
- limitations: `['BOUNDED_OCCUPIED_VOLUME_BUT_EMPIRICAL_NONCONSERVATIVE_ISOTROPIC_SIMILAR_SHAPE_SECOND_MOMENT_ESTIMATE_AFTER_NORMAL_OCCT_WORKER_FAILURE', 'dynamic_gripper_inertia_not_modeled', 'internal_component_mass_distribution_not_measured']`
- inertia tensor (kg·m²):

```text
[0.00045010858437538907, 5.2415202351444322e-06, -3.6621915682408926e-05]
[5.2415202351444322e-06, 0.00037548194776002844, -6.9808116983476422e-07]
[-3.6621915682408926e-05, -6.9808116983476422e-07, 0.00041731083802728472]
```
