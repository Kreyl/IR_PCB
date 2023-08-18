<map version="freeplane 1.9.8">
<!--To view this file, download free mind mapping software Freeplane from https://www.freeplane.org -->
<node TEXT="IR_PCB" FOLDED="false" ID="ID_1723255651" CREATED="1283093380553" MODIFIED="1691942870741"><hook NAME="MapStyle">
    <properties fit_to_viewport="false;" associatedTemplateLocation="template:/KL_standard-1.6.mm"/>

<map_styles>
<stylenode LOCALIZED_TEXT="styles.root_node" STYLE="oval" UNIFORM_SHAPE="true" VGAP_QUANTITY="24 pt">
<font SIZE="24"/>
<stylenode LOCALIZED_TEXT="styles.predefined" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="default" ID="ID_1285325844" MAX_WIDTH="600 px" COLOR="#000000" STYLE="bubble">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" DASH="" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_1285325844" STARTARROW="NONE" ENDARROW="DEFAULT"/>
<font NAME="SansSerif" SIZE="10" BOLD="false" ITALIC="false"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.details"/>
<stylenode LOCALIZED_TEXT="defaultstyle.attributes">
<font SIZE="9"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.note"/>
<stylenode LOCALIZED_TEXT="defaultstyle.floating">
<edge STYLE="hide_edge"/>
<cloud COLOR="#f0f0f0" SHAPE="ROUND_RECT"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.selection" BACKGROUND_COLOR="#4e85f8" BORDER_COLOR_LIKE_EDGE="false" BORDER_COLOR="#4e85f8"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.user-defined" POSITION="right" STYLE="bubble">
<stylenode TEXT="OK">
<icon BUILTIN="button_ok"/>
</stylenode>
<stylenode TEXT="NotOk">
<icon BUILTIN="button_cancel"/>
</stylenode>
<stylenode TEXT="Question">
<icon BUILTIN="help"/>
</stylenode>
<stylenode TEXT="Attention">
<icon BUILTIN="yes"/>
</stylenode>
<stylenode TEXT="Stopper">
<icon BUILTIN="stop-sign"/>
</stylenode>
<stylenode TEXT="Waiting">
<icon BUILTIN="hourglass"/>
</stylenode>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.AutomaticLayout" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="AutomaticLayout.level.root" COLOR="#000000">
<font SIZE="18"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,1" BACKGROUND_COLOR="#ccffcc"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,2" BACKGROUND_COLOR="#ffffcc"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,3" BACKGROUND_COLOR="#ffcccc"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,4" COLOR="#111111">
<font SIZE="10"/>
</stylenode>
</stylenode>
</stylenode>
</map_styles>
</hook>
<font SIZE="14" BOLD="true"/>
<node TEXT="Виды" POSITION="right" ID="ID_1728457538" CREATED="1691940003039" MODIFIED="1691940007997">
<node TEXT="Коптер" ID="ID_1343123522" CREATED="1691940008635" MODIFIED="1691940012761">
<node TEXT="Выстрел" ID="ID_1066567970" CREATED="1691942639892" MODIFIED="1691942643867">
<node TEXT="Вход1" ID="ID_566690111" CREATED="1691942367623" MODIFIED="1691942500895">
<node TEXT="Стреляет, пока 1 на входе" ID="ID_272750242" CREATED="1691940023164" MODIFIED="1691940047621">
<node TEXT="Частота 5Гц" ID="ID_1003961287" CREATED="1691942434844" MODIFIED="1691942465993"/>
</node>
</node>
<node TEXT="Вход2" ID="ID_1906467532" CREATED="1691942372770" MODIFIED="1691942503138">
<node TEXT="Стреляет один раз при появлении 1" ID="ID_1554156168" CREATED="1691942384613" MODIFIED="1691942398674">
<node TEXT="Для повторного выстрела нужно снова подать 1" ID="ID_958023998" CREATED="1691942398676" MODIFIED="1691942418022"/>
</node>
</node>
<node TEXT="Вспышка зелёным диодом" ID="ID_1608672174" CREATED="1691942668030" MODIFIED="1691942681100"/>
<node TEXT="Списывается один патрон из магазина" ID="ID_1818944602" CREATED="1691942650972" MODIFIED="1691942716411">
<node TEXT="Если патроны в магазине кончились" ID="ID_288372039" CREATED="1691942682512" MODIFIED="1691942707759">
<node TEXT="Быстро мигает одна красная диодная пара" ID="ID_1367114073" CREATED="1691942721006" MODIFIED="1691942745021"/>
<node TEXT="Начинается отсчёт задержки на смену магазина" ID="ID_1711785310" CREATED="1691942746066" MODIFIED="1691942806753">
<node TEXT="Если задержка кончилась" ID="ID_1810534195" CREATED="1691942779018" MODIFIED="1691942799693">
<node TEXT="Мигание диодной пары прекращается" ID="ID_1424659540" CREATED="1691942810823" MODIFIED="1691942827494"/>
<node TEXT="Если магазины кончились" ID="ID_1937943816" CREATED="1691942827905" MODIFIED="1691942835304">
<node TEXT="Светится одна красная диодная пара" ID="ID_1502205175" CREATED="1691942721006" MODIFIED="1692276403678"/>
<node TEXT="Пишется строка #MagazinesEnded" ID="ID_1896848432" CREATED="1691944272036" MODIFIED="1691944291569"/>
</node>
<node TEXT="Пишется строка #MagazineReloaded" ID="ID_12604984" CREATED="1691944055703" MODIFIED="1692367039297"/>
</node>
</node>
<node TEXT="Пишется строка &quot;#RoundsEnded&quot;" ID="ID_642561019" CREATED="1691942506759" MODIFIED="1691944244126"/>
</node>
</node>
</node>
<node TEXT="При попадании" ID="ID_573391003" CREATED="1691940151008" MODIFIED="1691940164538">
<node TEXT="Выдаётся импульс на Выход1" ID="ID_1220423459" CREATED="1691942477729" MODIFIED="1691942495948"/>
<node TEXT="Пишется строка &quot;#Hit from&quot; и указывается ID стрелявшего" ID="ID_1537716902" CREATED="1691942506759" MODIFIED="1692367071686"/>
<node TEXT="Вспышка всеми красными диодами" ID="ID_716892968" CREATED="1691942594206" MODIFIED="1691942622790"/>
<node TEXT="Списывается 1 хит" ID="ID_434071495" CREATED="1691942538572" MODIFIED="1691942581700">
<node TEXT="Если хиты кончились" ID="ID_1795977504" CREATED="1691942583749" MODIFIED="1691942590385">
<node TEXT="Выдаётся 1 на Выход2" ID="ID_1738167011" CREATED="1691942477729" MODIFIED="1691943512963"/>
<node TEXT="Пишется строка &quot;#HitsEnded&quot; в UART" ID="ID_190196755" CREATED="1691942506759" MODIFIED="1691943587991"/>
<node TEXT="Все красные диоды светятся" ID="ID_1499569371" CREATED="1691943591436" MODIFIED="1691943602380"/>
<node TEXT="Начинается отсчёт задержки на сброс" ID="ID_1618920581" CREATED="1691943603953" MODIFIED="1691943621263">
<node TEXT="Если задержка кончилась" ID="ID_637229962" CREATED="1691943622710" MODIFIED="1691943633844">
<node TEXT="Сброс (восстановление хитов, магазинов, патронов)" ID="ID_1182987067" CREATED="1691943636176" MODIFIED="1691943666827"/>
<node TEXT="Пишется строка #Restore" ID="ID_867483685" CREATED="1691943925074" MODIFIED="1691943957247"/>
</node>
</node>
</node>
</node>
</node>
</node>
<node TEXT="Пистолет" ID="ID_1491855374" CREATED="1691940012970" MODIFIED="1691940015519">
<node TEXT="Как коптер, но нет попаданий в силу отсутствия датчика" ID="ID_1502244170" CREATED="1691944352839" MODIFIED="1691944372753"/>
</node>
<node TEXT="Мишень" ID="ID_466319991" CREATED="1691940015726" MODIFIED="1691940020282">
<node TEXT="Как коптер, но не стреляет в силу отсутствия внешних сигналов на стрельбу" ID="ID_1432935346" CREATED="1691944374564" MODIFIED="1691944405469"/>
</node>
</node>
<node TEXT="Сущности" POSITION="right" ID="ID_1466762708" CREATED="1691942862706" MODIFIED="1691942865770">
<node TEXT="Патроны, они в магазинах" ID="ID_734245046" CREATED="1691943050018" MODIFIED="1691943062773">
<node TEXT="Патроны" ID="ID_1905654635" CREATED="1691942872733" MODIFIED="1691942875183">
<node TEXT="Патронов в магазине может быть от 0 до 254" ID="ID_1177763466" CREATED="1691942907917" MODIFIED="1691942926630">
<node TEXT="255 считается бесконечностью" ID="ID_416985859" CREATED="1691942927453" MODIFIED="1691942934975"/>
</node>
</node>
<node TEXT="Магазины" ID="ID_1071915011" CREATED="1691943067081" MODIFIED="1691943069722">
<node TEXT="Может быть от нуля до 254" ID="ID_1949438204" CREATED="1691943024660" MODIFIED="1691943042961">
<node TEXT="255 считается бесконечностью" ID="ID_1288277055" CREATED="1691942927453" MODIFIED="1691942934975"/>
</node>
</node>
</node>
<node TEXT="Хиты" ID="ID_318800845" CREATED="1691943021324" MODIFIED="1691943023619">
<node TEXT="Может быть от нуля до 254" ID="ID_752990946" CREATED="1691943024660" MODIFIED="1691943042961">
<node TEXT="255 считается бесконечностью" ID="ID_1025488537" CREATED="1691942927453" MODIFIED="1691942934975"/>
</node>
</node>
<node TEXT="Выстрел" ID="ID_1306694375" CREATED="1691943401872" MODIFIED="1691943405457">
<node TEXT="Несколько раз передаётся одинаковый ИК пакет" ID="ID_1693997369" CREATED="1691943406772" MODIFIED="1691943433120"/>
</node>
</node>
<node TEXT="Задержки" POSITION="right" ID="ID_915858402" CREATED="1691943676059" MODIFIED="1691943679536">
<node TEXT="Может быть от нуля до 254" ID="ID_582072650" CREATED="1691943024660" MODIFIED="1691943042961">
<node TEXT="255 считается бесконечностью" ID="ID_1797844658" CREATED="1691942927453" MODIFIED="1691942934975"/>
</node>
<node TEXT="В секундах" ID="ID_1965388071" CREATED="1691943712440" MODIFIED="1691943716508">
<node TEXT="На замену магазина" ID="ID_1797814698" CREATED="1691943165268" MODIFIED="1691943704808"/>
<node TEXT="На сброс после конца хитов" ID="ID_184879217" CREATED="1691943743554" MODIFIED="1691943752146"/>
</node>
</node>
<node TEXT="Схватка" POSITION="right" ID="ID_735404766" CREATED="1691944476595" MODIFIED="1691944667062">
<node TEXT="В одном зале может быть несколько схваток" ID="ID_841451097" CREATED="1691944671370" MODIFIED="1691944697496"/>
<node TEXT="У каждой должен быть свой ID" ID="ID_603589221" CREATED="1691944739027" MODIFIED="1691944749493">
<node TEXT="Значения" ID="ID_845109776" CREATED="1691944750617" MODIFIED="1691944757783">
<node TEXT="0, 1, 2, 3" ID="ID_1470263403" CREATED="1691944758853" MODIFIED="1691944766196"/>
</node>
</node>
<node TEXT="Пакеты из другой схватки игнорируются" ID="ID_1399963721" CREATED="1691944773836" MODIFIED="1691944793275">
<node TEXT="То есть, с другим ID схватки" ID="ID_133685578" CREATED="1691944794732" MODIFIED="1691944802401"/>
<node TEXT="В том числе пакет сброса" ID="ID_466825165" CREATED="1691945174377" MODIFIED="1691945194900">
<node TEXT="Чтоб судейский пакет из соседней схватки не влиял" ID="ID_455327535" CREATED="1691945197610" MODIFIED="1691945216211"/>
</node>
</node>
<node TEXT="Команда" ID="ID_1324749095" CREATED="1691944806913" MODIFIED="1691944812560">
<node TEXT="В каждой схватке несколько команд" ID="ID_675420204" CREATED="1691944813396" MODIFIED="1691944891638">
<node TEXT="От двух до восьми" ID="ID_60137926" CREATED="1691944920544" MODIFIED="1691944944193"/>
</node>
<node TEXT="У каждой должен быть свой ID" ID="ID_16315111" CREATED="1691944739027" MODIFIED="1691944749493">
<node TEXT="Значения" ID="ID_943845469" CREATED="1691944750617" MODIFIED="1691944757783">
<node TEXT="0, 1, 2, 3, 4, 5, 6, 7" ID="ID_1537853464" CREATED="1691944758853" MODIFIED="1691944953444"/>
</node>
</node>
<node TEXT="Пакеты от своей команды игнорируются" ID="ID_1256743485" CREATED="1691944773836" MODIFIED="1691944997468">
<node TEXT="То есть, с таким же ID команды" ID="ID_496411790" CREATED="1691944794732" MODIFIED="1691945012645"/>
</node>
</node>
</node>
<node TEXT="ИК-пакет" POSITION="right" ID="ID_1324919561" CREATED="1691945060388" MODIFIED="1691945066457">
<node TEXT="Длина 16 бит" ID="ID_50534348" CREATED="1691945067459" MODIFIED="1691945073997"/>
<node TEXT="Состав" ID="ID_1472601362" CREATED="1691945086426" MODIFIED="1691945096144">
<node TEXT="Тип" ID="ID_1819143967" CREATED="1691945097177" MODIFIED="1691945101616">
<node TEXT="Выстрел" ID="ID_1261593064" CREATED="1691945104250" MODIFIED="1691945106545"/>
<node TEXT="Сброс" ID="ID_1241412703" CREATED="1691945106776" MODIFIED="1691945109682">
<node TEXT="Например, судья может восстановить хиты и патроны выстрелом из своего пистолета" ID="ID_1419697848" CREATED="1691945114671" MODIFIED="1691945158038"/>
</node>
<node TEXT="3 бита" ID="ID_1229015537" CREATED="1691945442270" MODIFIED="1691945446047">
<node TEXT="Резерв на, может, лечилку или перезарядку" ID="ID_264488739" CREATED="1691945450672" MODIFIED="1691945467657"/>
</node>
</node>
<node TEXT="ID схватки" ID="ID_802889326" CREATED="1691945221729" MODIFIED="1691945231266">
<node TEXT="2 бита" ID="ID_1603172224" CREATED="1691945232361" MODIFIED="1691945234894"/>
</node>
<node TEXT="ID команды" ID="ID_1311940826" CREATED="1691945236455" MODIFIED="1691945243807">
<node TEXT="3 бита" ID="ID_975921243" CREATED="1691945261594" MODIFIED="1691945265056"/>
</node>
<node TEXT="N пакета" ID="ID_1457294184" CREATED="1691945277340" MODIFIED="1691945298440">
<node TEXT="Для определения, это новый пакет или повторный" ID="ID_729898293" CREATED="1691945298442" MODIFIED="1691945344631">
<node TEXT="В одном выстреле несколько одинаковых пакетов. В следующем выстреле N=N+1" ID="ID_1226771770" CREATED="1691947600503" MODIFIED="1691947644402"/>
<node TEXT="После получения пакета запускается задержка, чтобы избежать игнорирования нового пакета с таким же номером" ID="ID_1989448059" CREATED="1691946331865" MODIFIED="1691946369309"/>
</node>
<node TEXT="5 бит" ID="ID_398448746" CREATED="1691945929655" MODIFIED="1691946329052"/>
</node>
<node TEXT="Контрольная сумма" ID="ID_1795654623" CREATED="1691945351083" MODIFIED="1691945358627">
<node TEXT="3 бита" ID="ID_549753103" CREATED="1691946376949" MODIFIED="1691946379554">
<node TEXT="x^3 + x + 1" ID="ID_864151745" CREATED="1691946413142" MODIFIED="1691946424415">
<node TEXT="1011" OBJECT="java.lang.Long|1011" ID="ID_286970196" CREATED="1691946645626" MODIFIED="1691946647612"/>
</node>
</node>
</node>
</node>
</node>
</node>
</map>
