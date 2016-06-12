set nocompatible
let maplocalleader = "#"

syntax on
set expandtab
set softtabstop=2
set shiftwidth=2
set number
set nobackup
set noswapfile
set nowrap
set showmode
set tw=80
set smartcase
set smarttab
set smartindent
set autoindent
set incsearch
set mouse=a
set history=1000
set clipboard=unnamedplus,autoselect
filetype plugin indent on

set completeopt=menuone,menu,longest

set wildignore+=*\\tmp\\*,*.swp,*.swo,*.zip,.git,.cabal-sandbox
set wildmode=longest,list,full
set wildmenu
set completeopt+=longest

set t_Co=256

set cmdheight=1

execute pathogen#infect()

"python from powerline.vim import setup as powerline_setup
"python powerline_setup()
"python del powerline_setup

"set rtp+=/usr/local/lib/python3.5/dist-packages/powerline/bindings/vim/
"let g:syntastic_python_python_exec = 'python3'

map <Leader>s :SyntasticToggleMode<CR>

let g:syntastic_always_populate_loc_list = 1
let g:syntastic_auto_loc_list = 0
let g:syntastic_check_on_open = 0
let g:syntastic_check_on_wq = 0

let g:syntastic_cpp_compiler_options = '-std=c++14'

map <silent> tw :GhcModTypeInsert<CR>
map <silent> ts :GhcModSplitFunCase<CR>
map <silent> tq :GhcModType<CR>
map <silent> te :GhcModTypeClear<CR>

let g:SuperTabDefaultCompletionType = '<c-x><c-o>'

if has("gui_running")
  imap <c-space> <c-r>=SuperTabAlternateCompletion("\<lt>c-x>\<lt>c-o>")<CR>
else
  if has("unix")
    inoremap <Nul> <c-r>=SuperTabAlternationCompletion("<lt>c-x>\<lt>c-o>")<CR>
  endif
endif

let g:haskellmode_completion_ghc = 1
autocmd FileType haskell setlocal omnifunc=necoghc#omnifunc

map <Leader>n :NERDTreeToggle<CR>

let g:haskell_tabular = 1
vmap a= :Tabularize /=<CR>
vmap a; :Tabularize /::<CR>
vmap a- :Tabularize /-><CR>

map <silent> <Leader>t :CtrlP()<CR>
noremap <leader>b<space> :CtrlPBuffer<CR>
let g:ctrlp_custom_ignore = '\v[\/]dist$'

set laststatus=2 " Always display the statusline in all windows
set showtabline=2 " Always display the tabline, even if there is only one tab
set relativenumber
