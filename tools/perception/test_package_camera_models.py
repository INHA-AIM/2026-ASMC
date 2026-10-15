import unittest
from package_camera_models import inference_checkpoint


class PackageModelTest(unittest.TestCase):
    def test_ema_selected_and_custom_resume_fields_removed_without_mutating_source(self):
        ema, raw = object(), object()
        original = dict(ema=ema,model=raw,training_model=raw,epoch=7,
                        optimizer={'state':'large'},scheduler_state={'step':9},scaler={'amp':1})
        small = inference_checkpoint(original)
        self.assertIs(small['model'],ema)
        self.assertEqual(small['deployment_source_epoch'],7)
        self.assertEqual(small['epoch'],-1)
        self.assertNotIn('training_model',small)
        self.assertNotIn('scheduler_state',small)
        self.assertIsNone(small['optimizer'])
        self.assertEqual(original['optimizer'],{'state':'large'})

    def test_missing_model_rejected_and_pretrained_model_supported(self):
        with self.assertRaises(ValueError):
            inference_checkpoint({'model':None,'ema':None})
        model = object()
        self.assertIs(inference_checkpoint({'model':model,'epoch':-1})['model'],model)
