#!/usr/bin/env python3
"""Read-only v11 coverage, source restoration, evaluation exposure and E4 evidence."""
import argparse,json,html,shutil
from collections import Counter
from pathlib import Path
from PIL import Image,ImageDraw
from camera_pretrain_policy import CLASSES,sha
from prepare_camera_pretrain import read_rows


def report(a):
 rows=list(read_rows(a.dataset/'manifest.jsonl'));old=list(read_rows(a.source/'manifest.jsonl'));plan=json.loads((a.dataset/'restoration-plan.json').read_text());summary=json.loads((a.dataset/'summary.json').read_text())
 counts={role:dict(images=sum(r['split']==role for r in rows),completeness=Counter(r['supervision'] for r in rows if r['split']==role),positive=Counter(x['training_class'] for r in rows if r['split']==role for x in r['annotations'] if x['supervision']=='positive'),complete_positive=Counter(x['training_class'] for r in rows if r['split']==role and r['supervision']=='complete' for x in r['annotations'] if x['supervision']=='positive')) for role in ('train','selection','heldout')}
 coverage=[]
 for c in CLASSES:
  def n(rs,source=None):return sum(x['supervision']=='positive' and x['training_class']==c for r in rs if r['split']=='train' and (source is None or r['source'].startswith(source)) for x in r['annotations'])
  coverage.append(dict(class_name=c,old_train=n(old),train=n(rows),old_capture=n(old,'capture:'),capture=n(rows,'capture:'),val=counts['selection']['positive'][c],val_complete=counts['selection']['complete_positive'][c],test=counts['heldout']['positive'][c]))
 train=[r for r in rows if r['split']=='train'];patch=sum(r.get('map_drum_review',{}).get('added',0) for r in train)
 changes=dict(dataset=a.dataset.name,metadata_sha256=sha(a.dataset/'pretrain.json'),splits=counts,coverage=coverage,
     restored_comp_images=plan['restored_comp_train_images'],train_new_drum_boxes=patch,spatial_background_only_images=plan['spatial_background_only_images'],
     new_selection_seen_by_current_model=len(plan['new_selection_seen_by_current_model']),evaluation_limitations='Small sample or narrow source/scene coverage remains; partial frames contribute recall only.',training_started=False)
 (a.output/'coverage.json').write_text(json.dumps(changes,indent=2))
 for file in ('verification.json','restoration-plan.json'):shutil.copy2(a.dataset/file,a.output/file)
 media=a.output/'restored';media.mkdir(exist_ok=True);sample=[]
 for typ in ('Sedan','SUV','Wagon','MPV'):
  options=[]
  for r in train:
   if r['split_group']!='capture_scenario_family:comp_sample':continue
   original=r.get('provenance',{}).get('original_annotation',{}).get('annotations',[])
   matched=[x for x in r['annotations'] if x['supervision']=='positive' and x['training_class']=='car' and any(str(o.get('actor_id'))==str(x.get('actor_id')) and o.get('gt_source_label')==typ for o in original)]
   if matched:options.append((max((x['xyxy'][2]-x['xyxy'][0])*(x['xyxy'][3]-x['xyxy'][1]) for x in matched),r))
  if not options:continue
  _,r=max(options,key=lambda pair:pair[0]);im=Image.open(r['image']).convert('RGB');d=ImageDraw.Draw(im)
  for x in r['annotations']:
   if x['supervision']=='positive' and x['training_class']=='car':d.rectangle(x['xyxy'],outline='cyan',width=2)
  im.thumbnail((960,600));im.save(media/(typ+'.jpg'));sample.append(f'<figure><img loading="lazy" src="restored/{typ}.jpg"><figcaption>{typ} · {html.escape(r["id"])} · exact API vehicle model unknown</figcaption></figure>')
 body='<a href="../index.html">← ASMC 실험 목록</a><h1>v11 Capture 차량 복원·드럼 병합·배경 정정</h1><p>2026-10-13 · 데이터 준비 완료 / 실행 중 학습 교체 없음</p>'
 body+=f'<p class="note">Train {len(train):,}장 · selection {counts["selection"]["images"]}장 · heldout {counts["heldout"]["images"]}장. comp Capture {plan["restored_comp_train_images"]:,}장 복원. Heldout 이미지·주석은 v10 그대로입니다.</p>'
 body+='<h2>클래스 구성</h2><table><tr><th>클래스</th><th>이전 Train 박스</th><th>새 Train 박스</th><th>Capture 이전→현재</th><th>Val 전체 / complete</th><th>Test</th></tr>'
 for r in coverage:
  vals=[r['class_name'],r['old_train'],r['train'],f'{r["old_capture"]} → {r["capture"]}',f'{r["val"]} / {r["val_complete"]}',r['test']];body+='<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in vals)+'</tr>'
 body+='</table><p>Val의 partial 이미지는 recall·사례 검토용입니다. 누락 가능한 검출을 FP로 세지 않습니다. dog 등 20개 미만이거나 장면이 좁은 클래스는 소표본으로 표시하며 모든 종류의 독립 검증 완료로 주장하지 않습니다.</p>'
 body+=f'<h2>보강과 배경 수정</h2><p>새 train에 지도 고정 드럼 {patch:,}개를 병합했습니다. standing_object가 있는 이전 negative {plan["spatial_background_only_images"]:,}장은 partial로 바꿔 확인된 semantic 안전 셀만 배경 손실에 사용합니다. 기둥을 드럼으로 바꾼 것이 아닙니다. 잘못된 negative 5장은 제외하고 1장은 드럼 양성으로 보강했습니다.</p>'
 body+='<p>16종 ID, 4px/64px²(원본 또는 960 tensor), 기존 ignore·차체 mask·이미지당 최대2회 반복을 유지합니다. 복원된 작은 정상 양성도 같은 기준입니다. 원본 이미지·이전 버전은 변경하지 않았습니다.</p>'
 body+=f'<h2>이어 학습 여부</h2><p>새 selection {len(plan["new_selection_seen_by_current_model"])}장은 현재 v10 train에 포함됐습니다. 이어 학습 시 개발 진단으로만 해석해야 합니다. 새 분할에서 독립적인 selection 비교를 하려면 공식 사전학습 가중치부터 시작하는 것을 권장합니다. 아직 새 학습을 실행하지 않았습니다.</p>'
 efile=a.output/'epoch4-decision-evidence.json'
 if efile.exists():
  es=json.loads(efile.read_text())['evaluations'];body+='<h2>기존 평가본 E2 → E4</h2><p>confidence0.40 / 같은 기존 selection212. 새 v11 평가 결과가 아닙니다.</p><table><tr><th>Epoch</th><th>신규 macro F1</th><th>car TP</th><th>bus TP</th><th>person TP</th><th>motorcycle TP</th></tr>'
  for e in es:
   m=e['metrics'];vals=[e['epoch'],f'{m["supported_new_macro_f1"]:.4f}']+[f'{m["classes"][c]["tp"]}/{m["classes"][c]["gt"]}' for c in ('car','bus','person','motorcycle')];body+='<tr>'+''.join('<td>'+str(v)+'</td>' for v in vals)+'</tr>'
  body+='</table>'
 body+='<h2>복원 차량 예시</h2>'+''.join(sample)
 body+='<h2>평가 후보 개별 검수</h2><p>후보113장을 검토해 중복/불확실8장 보류, 불완전11장은 partial로 보존했습니다. 기존 complete100장과 새 complete94장, partial11장이 selection을 구성합니다.</p>'
 for file in sorted(a.output.glob('panel-*.jpg')):body+=f'<details><summary>{file.name}</summary><img loading="lazy" src="{file.name}"></details>'
 body+='<h2>검증 근거</h2><ul>'+''.join(f'<li><a href="{f}">{f}</a></li>' for f in ('coverage.json','verification.json','restoration-plan.json','reviews.json','epoch4-decision-evidence.json','readiness.json','loader-verification.json','final-verification.json','source-coverage/summary.json'))+'</ul>'
 style='body{font:16px/1.65 system-ui;background:#101a29;color:#e5edf6;max-width:1250px;margin:30px auto;padding:20px}a{color:#67d1f5}table{border-collapse:collapse;width:100%}td,th{padding:9px;border-bottom:1px solid #415064;text-align:left}img{max-width:100%}figure{margin:25px 0}figcaption{word-break:break-all}.note{padding:20px;background:#203149}h2{margin-top:35px}'
 (a.output/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>v11 Capture 차량 복원·데이터 재구성</title><style>'+style+'</style>'+body+'</html>')
 print(json.dumps(changes))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__)
 for name in ('dataset','source','output'):p.add_argument('--'+name,type=Path,required=True)
 report(p.parse_args())
