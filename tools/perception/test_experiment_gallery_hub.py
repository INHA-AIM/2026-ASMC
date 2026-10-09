import json
from pathlib import Path
import os
import tempfile
import unittest
from build_experiment_gallery_hub import catalogue, number_pages


class GalleryChronologyTest(unittest.TestCase):
    def test_identifiers_and_dates_survive_regeneration_and_removal(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); (root/'old').mkdir();(root/'new').mkdir()
            old=root/'old/index.html';old.write_text('<h1>Old</h1>');os.utime(old,(1000,1000))
            new=root/'new/index.html';new.write_text('<h1>New</h1>');os.utime(new,(2000,2000))
            def read(): return number_pages(root,catalogue(root,root/'index.html'))
            first=read();self.assertEqual('new/index.html',first[0]['path'])
            remembered={p['path']:(p['experiment_id'],p['timestamp']) for p in first}
            os.utime(old,(3000,3000))
            self.assertEqual(remembered,{p['path']:(p['experiment_id'],p['timestamp']) for p in read()})
            new.unlink();third=root/'new/third.html';third.write_text('<title>Third</title>')
            result=read();self.assertEqual('E003',result[0]['experiment_id'])
            self.assertIn('new/index.html',json.loads((root/'gallery-identifiers.json').read_text()))
            self.assertIn('KST',result[0]['title'])


if __name__=='__main__': unittest.main()
