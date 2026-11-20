# docs/tex 용 latexmk 설정. 한글본(paper_ko_*)은 fontspec 때문에 lualatex 가 필요하고 영문본(paper_en_*)은 pdflatex 로 만든다.
# 편집기(LaTeX Workshop 등)가 '-pdf' 를 붙여 호출해도 동작하도록, 한글본에서는 pdflatex 규칙이 실제로는 lualatex 를 실행하게 한다.
if (grep { /paper_ko_/ } @ARGV) {
	$pdflatex = 'lualatex %O %S';
	$pdf_mode = 1;
}
