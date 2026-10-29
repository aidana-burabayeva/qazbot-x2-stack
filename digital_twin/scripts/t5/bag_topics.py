import sqlite3, sys
for b in sys.argv[1:]:
    c = sqlite3.connect('file:%s?mode=ro' % b, uri=True)
    print(b)
    for r in c.execute('select t.name,t.type,count(m.id) from topics t left join messages m on m.topic_id=t.id group by t.id'):
        print('  ', r)
