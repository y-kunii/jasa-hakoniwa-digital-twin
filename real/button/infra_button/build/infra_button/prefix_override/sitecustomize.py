import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jasa/work/hakoniwa-digital-twin/real/button/infra_button/install/infra_button'
