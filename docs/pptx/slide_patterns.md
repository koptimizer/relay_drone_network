# 학술 발표용 슬라이드 패턴

필수 슬라이드 유형별 구현 패턴

모든 좌표는 `LAYOUT_16x9` (10" × 5.625") 기준. 다른 레이아웃은 비례 조정.

---

## 전역 기본값 (Global Defaults)

모든 슬라이드에 적용한다:

```javascript
const COLORS = {
  bg:       "FFFFFF",   // 흰 배경
  primary:  "1F4E79",   // 진네이비 — 제목
  accent:   "2E75B6",   // 미드블루 — 헤더, 강조
  body:     "2D2D2D",   // 근검정 — 본문
  muted:    "777777",   // 회색 — 인용, 캡션
  rule:     "CCCCCC",   // 연회색 — 구분선
  highlight:"FFF2CC",   // 노랑 — 콜아웃 박스 (정말 필요할 때만)
};

const FONTS = {
  face: "Aerial",   // 전체 단일 서체
  title: 16,             // 액션 타이틀: 22 pt
  sectionHeader: 14,     // 슬라이드 내 섹션 헤더
  body: 14,              // 본문 불릿: 최소 14 pt
  label: 12,             // 차트 주석, 인라인 라벨
  cite: 8,               // 슬라이드 내 인용, 각주
};

const MARGIN = 0.5;      // 슬라이드 가장자리 최소 여백 (인치)
```

> 참고: 본문이 16pt로 작으므로 텍스트 박스 높이를 원본보다 넉넉히 잡고, 슬라이드당 단어 수를 더 줄여 여백을 확보한다. Pretendard Light는 얇은 웨이트이므로 bold 강조가 상대적으로 잘 살아난다 — 핵심 어구에만 bold를 준다.

---

## 1. 제목 슬라이드 (Title Slide)

**목적:** 발표를 규정하고, 청중에게 완전한 출처 정보를 제공한다.

```javascript
slide.background = { color: COLORS.primary };

// 메인 타이틀 — 진술문 또는 질문형으로
slide.addText("수요·공급 불확실성 하 다계층 공급망 최적화를 위한 시나리오 MPC 접근", {
  x: 0.7, y: 1.4, w: 8.6, h: 1.8,
  fontSize: 30, fontFace: FONTS.face, color: "FFFFFF",
  bold: true, align: "left", valign: "top"
});

// 부제 / 학회 맥락
slide.addText("대한산업공학회 추계학술대회  ·  2026년 11월", {
  x: 0.7, y: 3.2, w: 8.6, h: 0.4,
  fontSize: 15, fontFace: FONTS.face, color: "A0BBDD",
  align: "left"
});

// 저자 및 소속
slide.addText("홍길동¹  ·  김철수²\n¹ OO대학교   ² OO연구소", {
  x: 0.7, y: 3.7, w: 8.6, h: 0.6,
  fontSize: 14, fontFace: FONTS.face, color: "CADCFC",
  align: "left"
});

```

**넣지 말 것:** 장식 이미지, 애니메이션 로고, 기관 문장(명시 요청 없으면), 클립아트.

---

## 2. 서론 슬라이드 (Research Background)

**목적:** 상황과 문제를 규정한다 — 왜 이 질문이 중요하고 무엇이 비어 있는가.

레이아웃: 2단(왼쪽 배경, 오른쪽 공백/문제) 또는 1단 서술.

```javascript
slide.addText("Introduction", {
  x: MARGIN, y: 0.2, w: 9.0, h: 0.8,
  fontSize: FONTS.title, fontFace: FONTS.face, color: COLORS.primary, bold: true, valign: "top"
});

// 제목 아래 얇은 구분선
slide.addShape(pres.shapes.RECTANGLE, {
  x: MARGIN, y: 1.0, w: 9.0, h: 0.025,
  fill: { color: COLORS.rule }
});

// 본문 불릿
slide.addText([
  { text: "다계층 공급망에서의 공급 불확실성: ", options: { bold: true, breakLine: false } },
  { text: "채찍 효과 등으로 인한 경제적 손실", options: { breakLine: true } },
], {
  x: MARGIN, y: 1.1, w: 9.0, h: 3.2,
  fontSize: FONTS.body, fontFace: FONTS.face, color: COLORS.body,
  bullet: true, paraSpaceAfter: 12
});

```

---

## 3. 연구 Gap 및 기여점

**목적:** 지금까지 연구와의 차이점 및 기여점 생성

```javascript
// 타이틀
slide.addText("Our contribution", {
  x: MARGIN, y: 0.2, w: 9.0, h: 0.9,
  fontSize: FONTS.title, fontFace: FONTS.face, color: COLORS.primary, bold: true
});

// 구분선
slide.addShape(pres.shapes.RECTANGLE, {
  x: MARGIN, y: 1.1, w: 9.0, h: 0.025, fill: { color: COLORS.rule }
});

slide.addText("불확실한 미래 수요를 고려한 공급망 최적화가 도움이 될 것인 지에 대해 질문하고 그에 대한 해결", {
  x: 1.7, y: 1.55, w: 6.6, h: 1.3,
  fontSize: 15, fontFace: FONTS.face, color: COLORS.primary,
  bold: false, align: "center", valign: "middle"
});
```

---

## 4. 방법론 (Methods)

**목적:** 제안하는 방법론의 핵심만 제공
목표: 1~2장. 청중에게 불필요한 세부는 모두 부록으로.

```javascript
slide.addText("Methodology", {
  x: MARGIN, y: 0.2, w: 9.0, h: 0.9,
  fontSize: FONTS.title, fontFace: FONTS.face, color: COLORS.primary, bold: true
});

// 문제 모델링 (페이지 1~n)
slide.addText("Problem Formulation ", {
  x: MARGIN, y: 1.25, w: 4.2, h: 0.35,
  fontSize: FONTS.sectionHeader, fontFace: FONTS.face, color: COLORS.accent, bold: true
});

slide.addText([
  { text: "상태: ", options: { bold: true, breakLine: false } },
  { text: "각 노드 재고 + context(위상·용량·비용·ADI)", options: { breakLine: true } },
  { text: "결정: ", options: { bold: true, breakLine: false } },
  { text: "생산량 y, 배송량 x (혼합 이산–연속)", options: { breakLine: true } },
  { text: "목적: ", options: { bold: true, breakLine: false } },
  { text: "backorder 허용 하 총운영비 최소화", options: { breakLine: true } },
], {
  x: MARGIN, y: 1.65, w: 4.2, h: 2.4,
  fontSize: FONTS.body, fontFace: FONTS.face, color: COLORS.body,
  bullet: true, paraSpaceAfter: 10
});

// 문제 해결 방법 (페이지 n~m)
slide.addText("Solution Approach", {
  x: 5.3, y: 1.25, w: 4.2, h: 0.35,
  fontSize: FONTS.sectionHeader, fontFace: FONTS.face, color: COLORS.accent, bold: true
});

slide.addText([
  { text: "Model Predictive Control 개념", options: { bold: true, breakLine: false } },
  { text: "고려하는 미래 시나리오 수에 기반한 모델 정확성 Proposition", options: { breakLine: true } },
], {
  x: MARGIN, y: 1.65, w: 4.2, h: 2.4,
  fontSize: FONTS.body, fontFace: FONTS.face, color: COLORS.body,
  bullet: true, paraSpaceAfter: 10
});
```

---

## 5. 결과 슬라이드 (Results)

**목적:** 제안하는 알고리즘의 장점, 연구의 finding 제공

```javascript
// 액션 타이틀 — 반드시 발견을 진술, 주제가 아니라
slide.addText("Comparative Study", {
  x: MARGIN, y: 0.2, w: 9.0, h: 0.85,
  fontSize: FONTS.title, fontFace: FONTS.face, color: COLORS.primary, bold: true
});

// 구분선
slide.addShape(pres.shapes.RECTANGLE, {
  x: MARGIN, y: 1.05, w: 9.0, h: 0.025, fill: { color: COLORS.rule }
});

// 실제 차트 또는 이미지로 생성
slide.addChart(pres.charts.BAR, [{
  name: "총운영비 (정규화)",
  labels: ["(s,S)", "greedy", "제안 정책"],
  values: [1.00, 0.82, 0.78, 0.61]
}], {
  x: MARGIN, y: 1.15, w: 5.4, h: 3.8,
  barDir: "col",
  chartColors: [COLORS.accent],
  chartArea: { fill: { color: COLORS.bg } },
  catAxisLabelColor: COLORS.muted,
  valAxisLabelColor: COLORS.muted,
  valGridLine: { color: "E2E8F0", size: 0.5 },
  catGridLine: { style: "none" },
  showValue: true,
  dataLabelColor: "1E293B",
  showLegend: false,
  valAxisTitle: "총운영비 (정규화)",
  showValAxisTitle: true,
  valAxisTitleColor: COLORS.muted,
  valAxisTitleFontSize: 11,
});



// 해석 텍스트 (다음 페이지)
slide.addText("Key finding", {
  x: 6.2, y: 1.15, w: 3.3, h: 0.35,
  fontSize: FONTS.sectionHeader, fontFace: FONTS.face, color: COLORS.accent, bold: true
});

slide.addText([
  { text: "평균 효과: 베이스라인 대비 비용 8% 절감", options: { breakLine: true } },
  { text: "backorder: 모든 소매점에서 0으로 수렴", options: { breakLine: true } },
], {
  x: 6.2, y: 1.55, w: 3.3, h: 2.5,
  fontSize: FONTS.body, fontFace: FONTS.face, color: COLORS.body,
  bullet: true, paraSpaceAfter: 12
});

```

**결과 슬라이드 필수 점검:**
- 핵심 한 문장으로부터 디테일이 꼬리를 물고 나가는 서술
- 최대한 간결한 단어 표현 위주
- 하단에 출처 인용

---

## 6. 참고문헌 슬라이드 (References)

**목적:** 모든 출처의 완전한 인용

```javascript
slide.background = { color: COLORS.bg };

slide.addText("참고문헌", {
  x: MARGIN, y: 0.2, w: 9.0, h: 0.5,
  fontSize: 22, fontFace: FONTS.face, color: COLORS.primary, bold: true
});

slide.addShape(pres.shapes.RECTANGLE, {
  x: MARGIN, y: 0.72, w: 9.0, h: 0.025, fill: { color: COLORS.rule }
});

const refs = [
  "Boute, R. et al. (2022). Deep reinforcement learning for inventory control. EJOR, 298(2), 401–412.",
  "Özer, Ö. & Wei, W. (2004). Inventory control with limited capacity and advance demand information. Operations Research, 52(6), 988–1000.",
  "Zeballos, L. et al. (2014). Addressing the uncertain quality and quantity of returns in closed-loop supply chains. Computers & Chemical Engineering, 47, 237–247.",
];

const refItems = refs.flatMap((r, i) => [
  { text: r, options: { breakLine: true } },
  ...(i < refs.length - 1 ? [{ text: "", options: { breakLine: true } }] : [])
]);

slide.addText(refItems, {
  x: MARGIN, y: 0.85, w: 9.0, h: 4.5,
  fontSize: 12, fontFace: FONTS.face, color: COLORS.body,
  paraSpaceAfter: 8
});
```

---

## 7. 부록 슬라이드 (Appendix)

**목적:** 미리 준비한 답변 슬라이드. Q&A 중 빠르게 이동할 수 있도록 명확히 라벨링한다.

```javascript
// 부록 라벨 — 본 덱과 구분되게 muted 처리
slide.addText("부록 B — 강건성 검증", {
  x: MARGIN, y: 0.15, w: 9.0, h: 0.4,
  fontSize: FONTS.label, fontFace: FONTS.face, color: COLORS.muted, bold: false, italics: true
});

// 이후 내용은 일반 슬라이드 패턴을 따름
```