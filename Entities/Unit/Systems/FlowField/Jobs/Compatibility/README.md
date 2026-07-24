# Compatibility types

This folder contains serialized data types retained so existing scenes and
debugger assets continue to deserialize. They are not runtime contact-pipeline
authorities and production modules must not call execution helpers from here.

Legacy Fat-AABB-era settings are translated into `EnablePersistentContactCache` and `PersistentGuardEnvelopeMargin`
once by `ContactPipelineConfiguration` into persistent-topology semantics.
