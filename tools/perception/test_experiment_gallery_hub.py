import json
from pathlib import Path
import os
import tempfile
import unittest
from build_experiment_gallery_hub import catalogue, number_pages, collect_pages, category, friendly_group


class GalleryChronologyTest(unittest.TestCase):
    def test_local2d_review_is_planning_without_implying_acceptance(self):
        self.assertEqual(category({'path':'local2d-run/index.html'}),'planning')
        self.assertIn('검수',friendly_group('local2d-run'))

    def test_source_snapshots_are_not_experiment_pages(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/"run/source-snapshot").mkdir(parents=True)
            (root/"run/source-snapshot/index.html").write_text("<title>source</title>")
            (root/"run/index.html").write_text("<title>result</title>")
            self.assertEqual(["run/index.html"], [p["path"] for p in catalogue(root, root/"index.html")])

    def test_partial_refresh_preserves_unselected_pages_and_titles(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            for group in ("old", "changed"):
                (root/group).mkdir()
                (root/group/"index.html").write_text("<title>"+group+"</title>")
            first=number_pages(root,catalogue(root,root/"index.html"))
            (root/"gallery-catalogue.json").write_text(json.dumps({"pages":first}))
            (root/"changed/index.html").unlink()
            (root/"changed/new.html").write_text("<title>new</title>")
            second=number_pages(root,collect_pages(root,root/"index.html",["changed"]))
            self.assertEqual({"old/index.html", "changed/new.html"}, {p["path"] for p in second})
            old1=next(p for p in first if p["group"]=="old")
            old2=next(p for p in second if p["group"]=="old")
            self.assertEqual(old1["experiment_id"],old2["experiment_id"])
            self.assertEqual(old1["title"],old2["title"])
            with self.assertRaises(ValueError):collect_pages(root,root/"index.html",["../bad"])

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
