%====================================================================================================
% Classe criada para padronizar as publicações produzidas no Campus CPCX-UFMS
%====================================================================================================
% Autor			: Kleber Kruger
% Referência	: Sergio Roberto de Freitas
% Instituição 	: UFMS - Universidade Federal do Mato Grosso do Sul
% Unidade		: CPCX - Campus de Coxim
%----------------------------------------------------------------------------------------------------
% Arquivo			: ufms-cpcx.cls
% Data de criação	: 22 de Setembro de 2014
%====================================================================================================

\NeedsTeXFormat{LaTeX2e}[2000/05/19]

\LoadClass[a4paper,12pt]{report}[2000/05/19 v1.4b Standard LaTeX document class]
\RequirePackage[utf8]{inputenc}
\RequirePackage[portuguese,brazil]{babel}

\RequirePackage{amsmath,amssymb,amsfonts,enumerate,float,fancyhdr,multirow,pifont}

%================================================================================
% Macro para dvi e pdf
%--------------------------------------------------------------------------------
\usepackage[pdftex]{graphicx}

%\usepackage[pdftex,pdfstartview=FitH,plainpages=true]{hyperref}
%\hypersetup{colorlinks=true,citecolor=black,filecolor=black,linkcolor=black,urlcolor=black}

% caminho dos arquivos gráficos
\graphicspath{{figuras/}}
% extensões que não serão necessárias especificar em toda instancia de \includegraphics
\DeclareGraphicsExtensions{.pdf,.jpeg,.png}

%================================================================================
% Configurando o layout da página
%--------------------------------------------------------------------------------
\setlength{\topmargin}{-7mm}
\setlength{\textheight}{235mm}
\setlength{\oddsidemargin}{0mm}
\setlength{\evensidemargin}{0mm}
\setlength{\textwidth}{160mm}
% Se não quiser a identação no início do parágrafo, descomente a linha abaixo:
%\setlength{\parindent}{0em}
\setlength{\parskip}{1.5ex}
\setlength{\headheight}{6mm}
%\addtolength{\headheight}{4pt} \tolerance=1000
\newcommand{\cmark}{\ding{51}}%
\newcommand{\xmark}{\ding{55}}%
\renewcommand{\chaptermark}[1]{\markboth{}{}}%
\lhead{\nouppercase{\quad\rightmark}\hfill{\sc CPCX-UFMS}}%
\rhead{\nouppercase{}}%
\renewcommand{\headrulewidth}{0.7pt}%
\pagestyle{fancy}
%==================== Fim da configuração do layout da página ===================

%================================================================================
% Definindo e customizando a página inicial
%--------------------------------------------------------------------------------
\newcommand{\titulo}[1]
{
	\begin{center}
		\begin{minipage}[t]{10cm}
			\begin{center}
				{{\Large #1}}
			\end{center}
		\end{minipage}
	\end{center}
}

\newcommand{\autor}[1]
{
	\relax\vskip 0.2cm\noindent
	\begin{center}
		\begin{minipage}[t]{10cm}
			\begin{center}
				{{\large #1}}
			\end{center}
		\end{minipage}
	\end{center}
}

\newcommand{\doctipo}[1]
{
	\relax\vskip 0.2cm\noindent
	\begin{center}
		\begin{minipage}[t]{10cm}
			\begin{center}
				{{#1}} %\quad {(#2)}
			\end{center}
		\end{minipage}
	\end{center}
}

\newcommand{\orientacao}[1]
{
	\vskip 2cm\noindent
	\begin{center}
		{Orienta\c{c}\~ao: #1}
	\end{center}
}

\newcommand{\docarea}[1]
{
	\vskip 0.2cm\noindent
	\begin{center}
		{\'{A}rea de Concentra\c{c}\~ao: #1}
	\end{center}
}

\newcommand{\textofree}[1]
{
	%\vskip 1cm
	\noindent
	\begin{center}
		{#1}
	\end{center}
}
%==================== Fim da customização da página inicial =====================

\setcounter{tocdepth}{2}% estabelece a "profundidade dos bookmarks".

\newtheorem{theorem}{Teorema}[chapter]
\newtheorem{definition}{Definição}[chapter]



%================================================================================
% Configurando códigos-fonte
%--------------------------------------------------------------------------------
\usepackage{listings}
%\usepackage{color}
%\usepackage{courier}

% Altera o nome padrão do rótulo usado no comando \autoref{}
\renewcommand{\lstlistingname}{Quadro}

% Altera o rótulo a ser usando no elemento pré-textual "Lista de código"
\renewcommand{\lstlistlistingname}{Lista de Quadros}

% Configura a "Lista de Códigos" conforme as regras da ABNT (para abnTeX2)
\begingroup\makeatletter
\let\newcounter\@gobble\let\setcounter\@gobbletwo
  \globaldefs\@ne \let\c@loldepth\@ne
%  \newlistof{listings}{lol}{\lstlistlistingname}
%  \newlistentry{lstlisting}{lol}{0}
\endgroup

%\renewcommand{\cftlstlistingaftersnum}{\hfill--\hfill}

\makeatletter
\let\my@chapter\@chapter
\renewcommand*{\@chapter}{%
	\addtocontents{lol}{\protect\addvspace{10pt}}%
	\my@chapter}
\makeatother

%\addcontentsline{toc}{section}{\protect\numberline{}{Listings}}

% Cria uma nova customização para a linguagem C++
\lstset{
	alsoother={0123456789_},
	backgroundcolor=\color{white},
	basicstyle=\ttfamily,columns=fullflexible,
	breakatwhitespace=false,
	breaklines=true,
	captionpos=b,
	escapeinside={\%*}{*)},
	extendedchars=true,
	frame=shadowbox,
	inputencoding=utf8,
	keepspaces=true,
	numberbychapter=false,
	numbers=left,
%	numbersep=10pt,
	showspaces=false,
	showstringspaces=false,
	showtabs=false,
	tabsize=4,
	framextopmargin=5pt,
	framexbottommargin=5pt,
	framexleftmargin=5pt,
	framexrightmargin=5pt,
	language=Java,
	literate={á}{{\'a}}1 {ã}{{\~a}}1 {é}{{\'e}}1 {è}{{\`{e}}}1 {ê}{{\^{e}}}1 {ë}{{\¨{e}}}1 {É}{{\'{E}}}1 {Ê}{{\^{E}}}1 {û}{{\^{u}}}1 {ú}{{\'{u}}}1 {â}{{\^{a}}}1 {à}{{\`{a}}}1 {á}{{\'{a}}}1 {ã}{{\~{a}}}1 {Á}{{\'{A}}}1 {Â}{{\^{A}}}1 {Ã}{{\~{A}}}1 {ç}{{\c{c}}}1 {Ç}{{\c{C}}}1 {õ}{{\~{o}}}1 {ó}{{\'{o}}}1 {ô}{{\^{o}}}1 {Õ}{{\~{O}}}1 {Ó}{{\'{O}}}1 {Ô}{{\^{O}}}1 {î}{{\^{i}}}1 {Î}{{\^{I}}}1 {í}{{\'{i}}}1 {Í}{{\~{Í}}}1
}
%==================== Fim da configuração dos códigos-fontes ====================
