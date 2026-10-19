#!/usr/bin/env python3
"""Build an offline source audit and reviewed map-drum gallery."""
import argparse,json,html,shutil
from pathlib import Path
from PIL import Image,ImageDraw

def main():
 p=argparse.ArgumentParser();p.add_argument('--report',type=Path,required=True);p.add_argument('--patch',type=Path,required=True);p.add_argument('--proposals',type=Path,required=True);a=p.parse_args()
 out=a.report;summary=json.loads((out/'summary.json').read_text());patch=json.loads((a.patch/'summary.json').read_text());sources=summary['source_class_split'];props={r['id']:r for r in map(json.loads,(a.proposals/'proposals.jsonl').open())};rows=list(map(json.loads,(a.patch/'manifest.jsonl').open()))
 media=out/'media';media.mkdir(exist_ok=True)
 # Original RGB and new boxes side by side; raw pixels untouched.
 chosen=[]
 for view in ['front','right']:
  for role in ['train','selection','heldout']:
   candidates=[r for r in rows if r['view']==view and r['split']==role]
   if candidates:chosen.append(max(candidates,key=lambda r:r['map_drum_review']['added']))
 gallery=[]
 for n,r in enumerate(chosen):
  raw=Image.open(r['image']).convert('RGB');after=raw.copy();d=ImageDraw.Draw(after)
  for ann in r['annotations'][r['map_drum_review']['original_annotation_count']:]:
   d.rectangle(ann['xyxy'],outline='cyan',width=2)
  tiles=[]
  for pic in [raw,after]:pic.thumbnail((640,400));tiles.append(pic)
  canvas=Image.new('RGB',(1280,430),'#101b27');d=ImageDraw.Draw(canvas)
  for i,tile in enumerate(tiles):canvas.paste(tile,(i*640,30));d.text((i*640+8,8),'Original RGB' if i==0 else 'Added traffic_drum · visible semantic components',fill='white')
  name=f'drum-{n:02}.jpg';canvas.save(media/name)
  gallery.append(f'<figure><a href="media/{name}"><img loading="lazy" src="media/{name}"></a><figcaption>{html.escape(r["id"])} · {r["split"]} · +{r["map_drum_review"]["added"]} · partial</figcaption></figure>')
 for file in (a.proposals/'review').glob('sheet-*.jpg'):shutil.copy2(file,media/file.name)
 old=out.parent/'camera-v10-vehicle-diagnostic-20261013-r1/car-source-sample.jpg'
 if old.exists():shutil.copy2(old,media/old.name)
 negative=Path('/tmp/map-drum-negatives.jpg')
 if negative.exists():shutil.copy2(negative,media/'incorrect-negatives.jpg')
 classrows=''.join('<tr><td>'+html.escape(c)+'</td>'+''.join(f'<td>{int(v.get(k,0)):,}</td>' for k in ['capture','roboflow','aim','bag'])+f'<td>{x["train_boxes"]:,}</td><td>{x["largest_source_share"]:.1%}</td></tr>' for c,x in summary['classes'].items() for v in [x['source_kind_boxes']])
 split=json.loads((out/'source-aware-split-proposal.json').read_text());types=json.loads((out/'comp-car-native-types.json').read_text())['counts']
 content=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>출처·차종 편중 재검토와 고주로 고정 드럼 보강</title>
 <style>body{{font:16px/1.6 system-ui;background:#0b1320;color:#e2eaf3;max-width:1280px;margin:auto;padding:28px}}a{{color:#62cafa}}h1{{font-size:30px}}h2{{margin-top:40px}}.note{{padding:18px;background:#18273b;border-left:4px solid #e8b85a}}table{{border-collapse:collapse;width:100%;margin:20px 0}}td,th{{text-align:left;padding:8px;border-bottom:1px solid #314158}}th{{color:#7dd3fc}}img{{max-width:100%}}figure{{margin:24px 0}}figcaption{{word-break:break-all;color:#a6bad1}}input,select{{padding:10px;background:#1a2a3c;color:white;border:1px solid #4e6178}}.scroll{{overflow:auto}}</style>
 <a href="../index.html">← ASMC 실험 목록</a><h1>출처·차종 편중 재검토와 고정 드럼 보강</h1><p>2026-10-13 · 원본/활성 학습 데이터 보존 · CPU 감사 · 학습 계속</p>
 <div class="note">차량 계보를 보존하는 분할에서 Capture car 대부분이 selection 예약으로 빠졌습니다. 이는 확인된 구성 문제이며, E2 회귀의 단일 인과관계를 증명한 것은 아닙니다. 현재 학습과 운영 모델은 변경하지 않았습니다.</div>
 <h2>현재 train 출처별 양성 박스</h2><p>이미지 수·박스 수는 차종 수가 아닙니다. 같은 배치의 여러 날씨도 독립 배치가 아닙니다.</p><div class="scroll"><table><thead><tr><th>클래스</th><th>Capture</th><th>Roboflow</th><th>AIM</th><th>bag</th><th>합계</th><th>최대 단일 소스 비중</th></tr></thead><tbody>{classrows}</tbody></table></div>
 <h2>차량 다양성 판정</h2><ul><li>car: Roboflow 11,400 / 11,449 (99.6%). 상위 aiv5 3,643·hlfma_3 3,463·jjj 2,037·mino 1,912박스. 정확한 차량 에셋명은 불명이며 원본 라벨이 car라는 사실만으로 다양성을 보장하지 않습니다.</li><li>comp_sample 예약 3,715장에는 car 양성 2,155개. Native GT 기준 {html.escape(str(types))}. Sedan/SUV/Wagon/MPV 외형 범주 다양성이 확인되지만 개별 API 차량 모델명은 미확인입니다.</li><li>bus: Capture 3,995개 중 Universe 2016 에셋 3,984개. 장수 증가와 에셋 다양성 증가를 구분해야 합니다.</li><li>truck: Capture 5,374개 중 HD65·Porter2·Xcient P520/P540·Mixer의 5에셋 5,373개 확인.</li><li>볼라드 87.7%, 드럼 60.6%가 각각 단일 소스에 집중. 차량만의 편중 문제가 아닙니다.</li></ul>
 <h2>고정 드럼: semantic + RGB 검수</h2><p>standing_object는 기둥·시설물도 포함하므로 전체를 드럼으로 변환하지 않았습니다. 위치, 색상, 연결 영역을 대조하고 소스별 최대 50장·총 637개 crop을 확인했습니다. 붙은 영역과 식별 불명은 보류합니다. Instance ID는 고정 맵 물체를 개별 구분하지 못했습니다.</p>
 <p><strong>{patch['images']:,}장 · {patch['added_boxes']:,}박스</strong> 보강. 현재 train 444장/+1,891, selection 예약 120장/+715, heldout 예약 297장/+1,282. 1,291개 초안 보류. 평가 예약 보강을 현재 평가 성적에 소급 적용하지 않았습니다.</p>
 <p>묶음 채택의 RGB 식별 기준 짧은 변 12px는 이번 자동 제안의 검수 신뢰도 제한입니다. 학습의 4px/64px² 기준을 바꾸거나 기존 작은 양성을 삭제한 것이 아닙니다. 모든 기존 주석과 분할을 보존했고 새 박스는 실제 visible semantic 영역입니다.</p>
 {''.join(gallery)}
 <h2>잘못된 배경 발견</h2><p>reviewed_negative 3,000장 전수 semantic 검사: 기존 target semantic은 0이지만 standing_object 픽셀이 2,945장, 16픽셀 이상 영역은 2,900장에 있습니다. 대부분 구조물일 수 있어 이 수를 오류 수로 세지 않습니다. RGB로 고정 드럼이 확인된 잘못된 negative는 현재 6장입니다. 1장은 partial 양성 보강, 나머지 5장은 새 버전에서 배경 제외 목록으로 기록했습니다. 현재 실행에는 미적용입니다.</p><img loading="lazy" src="media/incorrect-negatives.jpg"><p>확인한 partial 표본에서 드럼 영역의 safe background는 0이었습니다. 하지만 reviewed_negative는 전체 배경 손실이므로 이 6장의 누락은 억제 신호가 될 수 있습니다. 전체 회귀를 이 6장 탓으로 단정하지 않습니다.</p>
 <h2>재분할 준비 상태</h2><p>comp_sample을 train, comp_sample_b를 selection으로 옮기고 heldout을 유지하는 소스 보존안을 계산했습니다. 기존 pool의 selection dog≥20, Capture 차량별 train≥1,000을 동시에 만족하는 해는 없었습니다. dog≥10의 소표본 진단안만 생성했고 아직 데이터셋으로 채택하지 않았습니다.</p><p>진단안: train 후보 41,578장·car13,212박스 중 Capture2,155박스. selection pool2,412장/dog10박스. 실제 평가본은 별도 RGB 검수·분포 확인이 필요합니다. 이 안의 selection에는 현재 학습이 본 이미지가 있으므로 적용한다면 사전학습 가중치에서 새 실행이 필요합니다.</p>
 <p>우선 기존 comp 차량을 활용하고 배경 판정을 수정합니다. 추가 수집은 개별 차종을 확인한 뒤 부족한 외형·각도·원거리/FOGGY 조건과 독립 dog 평가 장면에 한정합니다. 지금 전체 재수집이 필요하다고 판단할 근거는 없습니다.</p>
 <h2>출처별 상세 조회</h2><select id="role"><option>train</option><option>selection</option><option>heldout</option></select> <select id="cls">{''.join('<option>'+c+'</option>' for c in summary['classes'])}</select> <input id="q" placeholder="출처 검색"><div class="scroll"><table><thead><tr><th>출처</th><th>이미지</th><th>양성 박스</th><th>분할 계보</th><th>알려진 하위 유형</th></tr></thead><tbody id="details"></tbody></table></div>
 <h2>검증·원본 근거</h2><p>6개 CPU 시험 통과. 실제 YOLO 파일·원본 SHA·기존 주석 보존·분할 계보·이미지 링크 검증 통과. 이 산출물은 label patch이며 standalone training dataset이 아닙니다.</p><ul><li><a href="summary.json">전체 출처 집계</a></li><li><a href="drum-patch-summary.json">드럼 보강 집계</a> · <a href="drum-verification.json">파일 재검증</a></li><li><a href="source-aware-strict-failure.json">엄격 분할 실패</a> · <a href="source-aware-split-proposal.json">소표본 진단 분할안</a></li><li><a href="negative-semantic-audit.json">배경 전수 검사</a></li><li><a href="../camera-v10-vehicle-diagnostic-20261013-r1/index.html">이전 차량 회귀 진단</a></li></ul><details><summary>637개 원본 검수 crop</summary>{''.join(f'<img loading="lazy" src="media/{x.name}">' for x in sorted(media.glob('sheet-*.jpg')))}</details>
 <script>const rows={json.dumps(sources,ensure_ascii=False).replace('<', chr(92)+'u003c')} ; const role=document.querySelector('#role'),cls=document.querySelector('#cls'),q=document.querySelector('#q');cls.value='car';function draw(){{let tbody=document.querySelector('#details');tbody.replaceChildren();for(let r of rows.filter(r=>r.split===role.value&&r.training_class===cls.value&&r.source.toLowerCase().includes(q.value.toLowerCase())).sort((a,b)=>b.boxes-a.boxes)){{let tr=document.createElement('tr');for(let v of [r.source,r.images,r.boxes,r.groups,JSON.stringify(r.subtypes)]){{let td=document.createElement('td');td.textContent=v;tr.append(td)}}tbody.append(tr)}}}}for(let el of [role,cls,q])el.addEventListener('input',draw);draw();</script></html>'''
 (out/'index.html').write_text(content)
 for name in ('summary.json','verification.json'):shutil.copy2(a.patch/name,out/('drum-patch-summary.json' if name=='summary.json' else 'drum-verification.json'))
 print(out/'index.html')
if __name__=='__main__':main()
